/*
 *  Copyright (C) 2002 Richard Henderson
 *  Copyright (C) 2001 Rusty Russell, 2002 Rusty Russell IBM.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <common.h>
#include <errno.h>
#include <module.h>
#include <elf.h>
#include <malloc.h>
#include <xfuncs.h>
#include <command.h>
#include <fs.h>
#include <kallsyms.h>

static unsigned long resolve_symbol(Elf32_Shdr *sechdrs, 
				    const char *name)
{
	unsigned long ret;

	debug("%s: %s\n", __FUNCTION__, name);
	ret = kallsyms_lookup_name(name);

	return ret;
}

/* Change all symbols so that sh_value encodes the pointer directly. */
static int simplify_symbols(Elf32_Shdr *sechdrs,
			    unsigned int symindex,
			    const char *strtab)
{
	Elf32_Sym *sym = (void *)sechdrs[symindex].sh_addr;
	unsigned long secbase;
	unsigned int i, n = sechdrs[symindex].sh_size / sizeof(Elf32_Sym);
	int ret = 0;

	for (i = 1; i < n; i++) {
		switch (sym[i].st_shndx) {
		case SHN_COMMON:
			/* We compiled with -fno-common.  These are not
			   supposed to happen.  */
			printf("Common symbol: %s\n", strtab + sym[i].st_name);
			printf("please compile with -fno-common\n");
			ret = -1;
			break;

		case SHN_ABS:
			/* Don't need to do anything */
			debug("Absolute symbol: 0x%08lx\n",
			       (long)sym[i].st_value);
			break;

		case SHN_UNDEF:
			sym[i].st_value
			  = resolve_symbol(sechdrs,
					   strtab + sym[i].st_name);
			debug("undef  : %20s 0x%08x 0x%08lx\n", strtab + sym[i].st_name, sym[i].st_value);

			/* Ok if resolved.  */
			if (sym[i].st_value != 0)
				break;
			/* Ok if weak.  */
			if (ELF32_ST_BIND(sym[i].st_info) == STB_WEAK)
				break;

			printf("Unknown symbol %s\n",
			       strtab + sym[i].st_name);
			ret = -1;
			break;

		default:
			secbase = sechdrs[sym[i].st_shndx].sh_addr;
			debug("default: %20s 0x%08x 0x%08lx\n", strtab + sym[i].st_name, sym[i].st_value, secbase);
			sym[i].st_value += secbase;
			break;
		}
	}

	return ret;
}

#define ALIGN(x,a)		__ALIGN_MASK(x,(typeof(x))(a)-1)
#define __ALIGN_MASK(x,mask)	(((x)+(mask))&~(mask))

/* Update size with this section: return offset. */
static long get_offset(unsigned long *size, Elf32_Shdr *sechdr)
{
	long ret;

	ret = ALIGN(*size, sechdr->sh_addralign ?: 1);
	*size = ret + sechdr->sh_size;
	return ret;
}

static void layout_sections( struct module *mod,
			    const Elf32_Ehdr *hdr,
			    Elf32_Shdr *sechdrs,
			    const char *secstrings)
{
	static unsigned long const masks[][2] = {
		/* NOTE: all executable code must be the first section
		 * in this array; otherwise modify the text_size
		 * finder in the two loops below */
		{ SHF_EXECINSTR | SHF_ALLOC },
		{ SHF_ALLOC, SHF_WRITE  },
		{ SHF_WRITE | SHF_ALLOC },
		{ SHF_ALLOC, 0 }
	};
	unsigned int m, i;

	for (i = 0; i < hdr->e_shnum; i++)
		sechdrs[i].sh_entsize = ~0UL;

	debug("Core section allocation order:\n");
	for (m = 0; m < ARRAY_SIZE(masks); ++m) {
		for (i = 0; i < hdr->e_shnum; ++i) {
			Elf32_Shdr *s = &sechdrs[i];

			if ((s->sh_flags & masks[m][0]) != masks[m][0]
			    || (s->sh_flags & masks[m][1])
			    || s->sh_entsize != ~0UL
			    || strncmp(secstrings + s->sh_name,
				       ".init", 5) == 0)
				continue;
			s->sh_entsize = get_offset(&mod->core_size, s);
			debug("\t%s 0x%08x\n", secstrings + s->sh_name, s->sh_entsize);
		}
	}
	debug("core_size: %ld\n", mod->core_size);
}

struct module * load_module(void *mod_image, unsigned long len)
{
	struct module *module = NULL;
	Elf32_Ehdr *ehdr;		/* Elf header structure pointer     */
	Elf32_Shdr *sechdrs;		/* Section header structure pointer */
	Elf32_Sym *sym;
	unsigned int numsyms;
	char *strtab = 0;		/* String table pointer             */
	int i;				/* Loop counter                     */
	unsigned int strindex = 0;
	unsigned int symindex = 0;
	char *secstrings;
	void *ptr = NULL;
	int err;

	if (len < sizeof(*ehdr))
		return NULL;

	ehdr = (Elf32_Ehdr *)mod_image;

	if (len < ehdr->e_shoff + ehdr->e_shnum * sizeof(Elf_Shdr))
		return NULL;

	module = xzalloc(sizeof(struct module));

	/* Find the section header string table for output info */
	sechdrs = (Elf32_Shdr *) (mod_image + ehdr->e_shoff +
			       (ehdr->e_shstrndx * sizeof (Elf32_Shdr)));

	if (sechdrs->sh_type == SHT_SYMTAB)
		strtab = (char *) (mod_image + sechdrs->sh_offset);

	/* Convenience variables */
	sechdrs = (void *)ehdr + ehdr->e_shoff;
	secstrings = (void *)ehdr + sechdrs[ehdr->e_shstrndx].sh_offset;
	sechdrs[0].sh_addr = 0;

	for (i = 0; i < ehdr->e_shnum; i++) {

		debug("%d addr: 0x%08x size: 0x%08x ofs: 0x%08x\n",
				i, sechdrs[i].sh_addr, sechdrs[i].sh_size, sechdrs[i].sh_offset);

		/* Mark all sections sh_addr with their address in the
		   temporary image. */
		sechdrs[i].sh_addr = (size_t)ehdr + sechdrs[i].sh_offset;

		if (sechdrs[i].sh_type == SHT_SYMTAB) {
			symindex = i;
			strindex = sechdrs[i].sh_link;
			strtab = mod_image + sechdrs[strindex].sh_offset;
		}
	}
	if (symindex == 0) {
		printf("module has no symbols (stripped?)\n");
		err = -ENOEXEC;
		goto cleanup;
	}

	/* Determine total sizes, and put offsets in sh_entsize.  For now
	   this is done generically; there doesn't appear to be any
	   special cases for the architectures. */
	layout_sections(module, ehdr, sechdrs, secstrings);

	ptr = xzalloc(module->core_size);
	module->module_core = ptr;

	/* Transfer each section which specifies SHF_ALLOC */
	debug("final section addresses:\n");
	for (i = 0; i < ehdr->e_shnum; i++) {
		void *dest;

		if (!(sechdrs[i].sh_flags & SHF_ALLOC))
			continue;

		dest = module->module_core + sechdrs[i].sh_entsize;
		debug("0x%08x dest 0x%p\n", sechdrs[i].sh_addr, dest);

		if (sechdrs[i].sh_type != SHT_NOBITS)
			memcpy(dest, (void *)sechdrs[i].sh_addr,
			       sechdrs[i].sh_size);
		/* Update sh_addr to point to copy in image. */
		sechdrs[i].sh_addr = (unsigned long)dest;
	}

	/* Fix up syms, so that st_value is a pointer to location. */
	err = simplify_symbols(sechdrs, symindex, strtab);
	if (err < 0)
		goto cleanup;

	for (i = 0; i < ehdr->e_shnum; i++) {
		if (sechdrs[i].sh_type == SHT_REL) {
			apply_relocate(sechdrs, strtab, symindex, i, module);
		}
		if (sechdrs[i].sh_type == SHT_RELA)
			apply_relocate_add(sechdrs, strtab, symindex, i, module);
	}

	numsyms = sechdrs[symindex].sh_size / sizeof(Elf32_Sym);
	sym = (void *)sechdrs[symindex].sh_addr;

	for (i = 0; i < numsyms; i++) {
		if (!strcmp(strtab + sym[i].st_name, MODULE_SYMBOL_PREFIX "init_module")) {
			printf("found init_module() at 0x%08x\n", sym[i].st_value);
			module->init = (void *)sym[i].st_value;
		}
	}

	return module;

cleanup:
	if (ptr)
		free(ptr);
	if (module)
		free(module);

	return NULL;
}

static int do_insmod (cmd_tbl_t *cmdtp, int argc, char *argv[])
{
	struct module *module;
	void *buf;
	int len;

	if (argc < 2) {
		u_boot_cmd_usage(cmdtp);
		return 1;
	}

	buf = read_file(argv[1], &len);
	if (!buf) {
		printf("error\n");
		return 1;
	}

	module = load_module(buf, len);

	free(buf);

#ifdef CONFIG_BLACKFIN
	/*
	 * FIXME: We need this for blackfin to disable the protection unit.
	 *        This is ok for first simple testing, but what we really
	 *        need is a mprotect call.
	 */
	icache_disable();
#endif

	if (module) {
		if (module->init)
			module->init();
	}

#ifdef  CONFIG_BLACKFIN
	icache_enable();
#endif

	return 0;
}

static __maybe_unused char cmd_insmod_help[] =
"Usage: insmod <module>\n"; 

U_BOOT_CMD_START(insmod)
	.maxargs	= 2,
	.cmd		= do_insmod,
	.usage		= "insert a module",
	U_BOOT_CMD_HELP(cmd_insmod_help)
U_BOOT_CMD_END
