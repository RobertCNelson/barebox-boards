/*
 * Copyright (C) 2013 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * Under GPLv2
 */

#include <common.h>
#include <init.h>
#include <clock.h>
#include <driver.h>
#include <xfuncs.h>
#include <errno.h>
#include <param.h>
#include <fcntl.h>
#include <malloc.h>
#include <io.h>
#include <linux/clk.h>
#include <linux/err.h>

#define FUSE_CR		0x00			/* Fuse Control Register */
#define		FUSE_CR_WRQ		(1 << 0)		/* Write Request */
#define		FUSE_CR_RRQ		(1 << 1)		/* Read Request */
#define		FUSE_CR_KEY		0xfb			/* Key code */
#define FUSE_MR		0x04			/* Fuse Mode Register */
#define FUSE_IR		0x08			/* Fuse Index Register */
#define		FUSE_IR_WS		(1 << 0)		/* Write Status */
#define		FUSE_IR_RS		(1 << 1)		/* Read Status */
#define		FUSE_IR_WSEL(x)		(((x) & 0xf) << 8)	/* Word Selection */
#define FUSE_DR		0x0C			/* Fuse Data Register */
#define FUSE_SR(x)	(0x10 + ((x) * 4))	/* Fuse Status Register x */

struct at91_fuse {
	struct cdev cdev;
	void __iomem *base;
	uint8_t *data;
	int nb_reg;
	bool is_read;
};

static int at91_fuse_read(struct at91_fuse *priv)
{
	int ret;
	int i;
	uint32_t *buf = (uint32_t*)priv->data;

	__raw_writel(FUSE_CR_KEY | FUSE_CR_RRQ, priv->base + FUSE_CR);

	ret = wait_on_timeout(100 * MSECOND,
		__raw_readl(priv->base + FUSE_IR) & FUSE_IR_RS);
	if (ret)
		return ret;

	for (i = 0; i < priv->nb_reg; i++)
		buf[i] = __raw_readl(priv->base + FUSE_SR(i));

	priv->is_read = true;

	return 0;
}

static int at91_fuse_write(struct at91_fuse *priv)
{
	int ret;
	int i;
	uint32_t *buf = (uint32_t*)priv->data;

	ret = wait_on_timeout(100 * MSECOND,
		__raw_readl(priv->base + FUSE_IR) & (FUSE_IR_RS | FUSE_IR_WS));
	if (ret)
		return ret;

	for (i = 0; i < priv->nb_reg; i++) {
		__raw_writel(FUSE_IR_WSEL(i), priv->base + FUSE_IR);
		__raw_writel(buf[i], priv->base + FUSE_DR);
		__raw_writel(FUSE_CR_KEY | FUSE_CR_WRQ, priv->base + FUSE_CR);
		ret = wait_on_timeout(100 * MSECOND,
			__raw_readl(priv->base + FUSE_IR) & FUSE_IR_WS);
		if (ret)
			return ret;
	}

	return 0;
}

static ssize_t at91_fuse_cdev_read(struct cdev *cdev, void *buf, size_t count,
		loff_t offset, ulong flags)
{
	struct at91_fuse *priv = cdev->priv;

	if (!priv->is_read) {
		int ret;

		ret = at91_fuse_read(priv);
		if (ret)
			return ret;
	}

	memcpy(buf, &priv->data[offset], count);

	return count;
}

static ssize_t at91_fuse_cdev_write(struct cdev *cdev, const void *buf, size_t count,
		loff_t offset, ulong flags)
{
	struct at91_fuse *priv = cdev->priv;

	if (!priv->is_read) {
		int ret;

		ret = at91_fuse_read(priv);
		if (ret)
			return ret;
	}

	memcpy(&priv->data[offset], buf, count);

	return count;
}

static struct file_operations at91_fuse_ops = {
	.read	= at91_fuse_cdev_read,
	.write	= at91_fuse_cdev_write,
	.lseek	= dev_lseek_default,
};

static int at91_fuse_write_set(struct device_d *dev, struct param_d *param,
		const char *val)
{
	struct at91_fuse *priv = dev->priv;
	unsigned long blow_enable;

	if (val == NULL)
		return -EINVAL;

	blow_enable = simple_strtoul(val, NULL, 0);
	if (blow_enable > 1)
		return -EINVAL;

	if (blow_enable) {
		int ret;

		ret = at91_fuse_write(priv);
		if (ret)
			return ret;
	}

	return dev_param_set_generic(dev, param, blow_enable ? "1" : "0");
}

static int at91_fuse_probe(struct device_d *dev)
{
	struct at91_fuse *priv;
	struct cdev *cdev;
	int ret;
	int nb = (int)dev->platform_data;
	struct clk *clk;

	if (!nb) {
		dev_err(dev, "missing number of fuse data register");
		return -EIO;
	}

	clk = clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "clock not found: %d\n", ret);
		return ret;
	}

	ret = clk_enable(clk);
	if (ret < 0) {
		dev_err(dev, "clock failed to enable: %d\n", ret);
		goto err_clk;
	}

	priv = xzalloc(sizeof(*priv));

	dev->priv = priv;

	priv->base = dev_request_mem_region(dev, 0);
	priv->nb_reg = nb;
	priv->data = xzalloc(sizeof(uint32_t) * nb);

	cdev = &priv->cdev;
	cdev->dev = dev;
	cdev->ops = &at91_fuse_ops;
	cdev->priv = priv;
	cdev->size = 4 * nb;
	cdev->name = xstrdup(dev_name(dev));

	ret = devfs_create(cdev);
	if (ret)
		goto err_devfs;

	ret = dev_add_param(dev, "permanent_write",
			at91_fuse_write_set, NULL, 0);
	if (ret)
		goto err_add_param;
	ret = dev_set_param(dev, "permanent_write", "0");
	if (ret)
		goto err_set_param;

	return 0;

err_set_param:
	dev_remove_param(dev, "permanent_write");
err_add_param:
	devfs_remove(cdev);
err_devfs:
	free(cdev->name);
	free(priv);
	clk_disable(clk);
err_clk:
	clk_put(clk);
	return ret;
}

static struct driver_d at91_fuse_driver = {
	.name	= "at91-fuse",
	.probe	= at91_fuse_probe,
};

static int at91_fuse_init(void)
{
	platform_driver_register(&at91_fuse_driver);

	return 0;
}
coredevice_initcall(at91_fuse_init);
