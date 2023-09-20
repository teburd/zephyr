/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "zephyr/sys/slist.h"
#include "zephyr/sys/util.h"
#include <zephyr/modules/elf.h>
#include <zephyr/modules/module.h>
#include <zephyr/modules/buf_stream.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modules, CONFIG_MODULES_LOG_LEVEL);

#include <string.h>

static struct module_symtable SYMTAB;


/** Default heap for metadata and module regions */
K_HEAP_DEFINE(module_heap, CONFIG_MODULES_HEAP_SIZE * 1024);

/* With MMU/MPU and memory protection different allocators are needed
 * for different pages and permissions.
 */
#ifdef CONFIG_MODULES_MEMPROTECTED
struct k_heap module_rx_heap;
struct k_heap module_rw_heap;
struct k_heap module_ro_heap;

#define MODULE_RX_HEAP module_rx_heap
#define MODULE_RW_HEAP module_rw_heap
#define MODULE_RO_HEAP module_ro_heap

#else

#define MODULE_RX_HEAP module_heap
#define MODULE_RW_HEAP module_heap
#define MODULE_RO_HEAP module_heap

#endif /* CONFIG_MODULES_MEM_PROTECTED */

static inline void *module_alloc_rx(size_t sz)
{
	return k_heap_alloc(&MODULE_RX_HEAP, sz, K_NO_WAIT);
}

static inline void module_free_rx(void *p)
{
	k_heap_free(&MODULE_RX_HEAP, p);
}

static inline void *module_alloc_rw(size_t sz)
{
	return k_heap_alloc(&MODULE_RW_HEAP, sz, K_NO_WAIT);
}

static inline void module_free_rw(void *p)
{
	k_heap_free(&MODULE_RW_HEAP, p);
}

static inline void *module_alloc_ro(size_t sz)
{
	return k_heap_alloc(&MODULE_RO_HEAP, sz, K_NO_WAIT);
}

static inline void module_free_ro(void *p)
{
	k_heap_free(&MODULE_RO_HEAP, p);
}

static inline void *module_mem_alloc(enum module_mem m, size_t sz)
{
	void *mem = NULL;

	switch (m) {
	case MOD_MEM_TEXT:
		mem = module_alloc_rx(sz);
		break;
	case MOD_MEM_RODATA:
		mem = module_alloc_ro(sz);
		break;
	default:
		mem = module_alloc_rw(sz);
		break;
	}
	return mem;
}


static inline void module_mem_free(enum module_mem m, void *p)
{
	switch (m) {
	case MOD_MEM_TEXT:
		module_free_rx(p);
		break;
	case MOD_MEM_RODATA:
		module_free_ro(p);
		break;
	default:
		module_free_rw(p);
		break;
	}
}

static const char ELF_MAGIC[] = {0x7f, 'E', 'L', 'F'};

int module_buf_read(struct module_stream *s, void *buf, size_t len)
{
	struct module_buf_stream *buf_s = CONTAINER_OF(s, struct module_buf_stream, stream);
	size_t end = MIN(buf_s->pos + len, buf_s->len);
	size_t read_len = end - buf_s->pos;

	memcpy(buf, buf_s->buf + buf_s->pos, read_len);
	buf_s->pos = end;

	return read_len;
}

int module_buf_seek(struct module_stream *s, size_t pos)
{
	struct module_buf_stream *buf_s = CONTAINER_OF(s, struct module_buf_stream, stream);

	buf_s->pos = MIN(pos, buf_s->len);

	return 0;
}

int module_read(struct module_stream *s, void *buf, size_t len)
{
	return s->read(s, buf, len);
}

int module_seek(struct module_stream *s, size_t pos)
{
	return s->seek(s, pos);
}

static sys_slist_t _module_list = SYS_SLIST_STATIC_INIT(&_module_list);

sys_slist_t *module_list(void)
{
	return &_module_list;
}

struct module *module_from_name(const char *name) {
	sys_slist_t *mlist = module_list();
	sys_snode_t *node = sys_slist_peek_head(mlist);
	struct module *m = CONTAINER_OF(node, struct module, _mod_list);

	while (node != NULL) {
		if (strncmp(m->name, name, sizeof(m->name)) == 0){
			return m;
		}
		node = sys_slist_peek_next(node);
		m = CONTAINER_OF(node, struct module, _mod_list);
	}

	return NULL;
}

/* find arbitrary symbol's address according to its name in a module */
void *module_find_sym(const struct module_symtable *sym_table, const char *sym_name)
{
	elf_word i;

	/* find symbols in module */
	for (i = 0; i < sym_table->sym_cnt; i++) {
		if (strcmp(sym_table->syms[i].name, sym_name) == 0) {
			LOG_DBG("Found name %s at %p",
				sym_table->syms[i].name, sym_table->syms[i].addr);
			return sym_table->syms[i].addr;
		}
	}

	LOG_DBG("Symbol \"%s\" not found", sym_name);

	return NULL;
}

static void module_link_plt(struct module_stream *ms, struct module *m, elf_shdr_t *shdr)
{
	unsigned int sh_cnt = shdr->sh_size / shdr->sh_entsize;
	/*
	 * CPU address where the .text section is stored, we use .text just as a
	 * reference point
	 */
	uint8_t *text = module_peek(ms, ms->sects[MOD_SECT_TEXT].sh_offset);

	LOG_DBG("Found %p in PLT %u size %u cnt %u text %p",
		(void *)module_peek(ms, ms->sects[MOD_SECT_SHSTRTAB].sh_offset + shdr->sh_name),
		shdr->sh_type, shdr->sh_entsize, sh_cnt, (void *)text);

	const elf_shdr_t *sym_shdr = ms->sects + MOD_SECT_SYMTAB;
	unsigned int sym_cnt = sym_shdr->sh_size / sym_shdr->sh_entsize;

	for (unsigned int i = 0; i < sh_cnt; i++) {
		elf_rela_t *rela = module_peek(ms, shdr->sh_offset + i * shdr->sh_entsize);
		/* Index in the symbol table */
		int j = ELF32_R_SYM(rela->r_info);

		if (j >= sym_cnt) {
			LOG_WRN("PLT: idx %u >= %u", j, sym_cnt);
			continue;
		}

		elf_sym_t *sym_tbl = module_peek(ms, sym_shdr->sh_offset +
						 j * sizeof(elf_sym_t));
		uint32_t stt = ELF_ST_TYPE(sym_tbl->st_info);
		char *name = module_peek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset +
					 sym_tbl->st_name);
		/*
		 * Both r_offset and sh_addr are addresses for which the module
		 * has been built.
		 */
		size_t got_offset = rela->r_offset - ms->sects[MOD_SECT_TEXT].sh_addr;

		if (stt == STT_NOTYPE && sym_tbl->st_shndx == SHN_UNDEF && name[0] != '\0') {
			void *link_addr = module_find_sym(&SYMTAB, name);

			if (!link_addr) {
				LOG_WRN("PLT: cannot find idx %u name %s", j, name);
				continue;
			}

			if (!rela->r_offset) {
				LOG_WRN("PLT: zero offset idx %u name %s", j, name);
				continue;
			}

			LOG_DBG("symbol %s offset %#x r-offset %#x .text offset %#x",
				name, got_offset,
				rela->r_offset, ms->sects[MOD_SECT_TEXT].sh_addr);

			/* Resolve the symbol */
			*(void **)(text + got_offset) = link_addr;
		}
	}
}

__weak void arch_elf_relocate(elf_rel_t *rel, uintptr_t opaddr, uintptr_t opval)
{
}

/**
 * @brief load a relocatable object file.
 *
 * An unlinked or partially linked elf will have symbols that have yet to be
 * determined and must be linked in effect. This is similiar, but not exactly like,
 * a dynamic elf. Typically the code and addresses *are* position dependent.
 */
static int module_load_rel(struct module_stream *ms, struct module *m)
{
	elf_word i, j, sym_cnt, rel_cnt;
	elf_rel_t rel;
	char name[32];

	m->mem_size = 0;
	m->sym_tab.sym_cnt = 0;

	elf_shdr_t shdr;
	size_t pos = ms->hdr.e_shoff;
	unsigned int str_cnt = 0;

	ms->sect_map = k_heap_alloc(&module_heap, ms->hdr.e_shnum * sizeof(uint32_t), K_NO_WAIT);
	if (!ms->sect_map)
		return -ENOMEM;
	ms->sect_cnt = ms->hdr.e_shnum;

	ms->sects[MOD_SECT_SHSTRTAB] =
		ms->sects[MOD_SECT_STRTAB] =
		ms->sects[MOD_SECT_SYMTAB] = (elf_shdr_t){0};

	/* Find symbol and string tables */
	for (i = 0; i < ms->hdr.e_shnum && str_cnt < 3; i++) {
		module_seek(ms, pos);
		module_read(ms, &shdr, sizeof(elf_shdr_t));

		pos += ms->hdr.e_shentsize;

		LOG_DBG("section %d at %x: name %d, type %d, flags %x, addr %x, size %d",
			i,
			ms->hdr.e_shoff + i * ms->hdr.e_shentsize,
			shdr.sh_name,
			shdr.sh_type,
			shdr.sh_flags,
			shdr.sh_addr,
			shdr.sh_size);

		switch (shdr.sh_type) {
		case SHT_SYMTAB:
		case SHT_DYNSYM:
			LOG_DBG("symtab at %d", i);
			ms->sects[MOD_SECT_SYMTAB] = shdr;
			ms->sect_map[i] = MOD_SECT_SYMTAB;
			str_cnt++;
			break;
		case SHT_STRTAB:
			if (ms->hdr.e_shstrndx == i) {
				LOG_DBG("shstrtab at %d", i);
				ms->sects[MOD_SECT_SHSTRTAB] = shdr;
				ms->sect_map[i] = MOD_SECT_SHSTRTAB;
			} else {
				LOG_DBG("strtab at %d", i);
				ms->sects[MOD_SECT_STRTAB] = shdr;
				ms->sect_map[i] = MOD_SECT_STRTAB;
			}
			str_cnt++;
			break;
		default:
			break;
		}
	}

	if (!ms->sects[MOD_SECT_SHSTRTAB].sh_type ||
	    !ms->sects[MOD_SECT_STRTAB].sh_type ||
	    !ms->sects[MOD_SECT_SYMTAB].sh_type) {
		LOG_ERR("Some sections are missing or present multiple times!");
		return -ENOENT;
	}

	pos = ms->hdr.e_shoff;

	/* Copy over useful sections */
	for (i = 0; i < ms->hdr.e_shnum; i++) {
		module_seek(ms, pos);
		module_read(ms, &shdr, sizeof(elf_shdr_t));

		pos += ms->hdr.e_shentsize;

		elf32_word str_idx = shdr.sh_name;

		module_seek(ms, ms->sects[MOD_SECT_SHSTRTAB].sh_offset + str_idx);
		module_read(ms, name, sizeof(name));
		name[sizeof(name) - 1] = '\0';

		LOG_DBG("section %d name %s", i, name);

		enum module_mem mem_idx;
		enum module_section sect_idx;

		if (strncmp(name, ".text", sizeof(name)) == 0) {
			mem_idx = MOD_MEM_TEXT;
			sect_idx = MOD_SECT_TEXT;
		} else if (strncmp(name, ".data", sizeof(name)) == 0) {
			mem_idx = MOD_MEM_DATA;
			sect_idx = MOD_SECT_DATA;
		} else if (strncmp(name, ".rodata", sizeof(name)) == 0) {
			mem_idx = MOD_MEM_RODATA;
			sect_idx = MOD_SECT_RODATA;
		} else if (strncmp(name, ".bss", sizeof(name)) == 0) {
			mem_idx = MOD_MEM_BSS;
			sect_idx = MOD_SECT_BSS;
		} else if (strncmp(name, ".module", sizeof(name)) == 0) {
			m->module_offset = shdr.sh_offset;
			continue;
		} else {
			LOG_DBG("Not copied section %s", name);
			continue;
		}

		ms->sects[sect_idx] = shdr;
		ms->sect_map[i] = sect_idx;

		m->mem[mem_idx] =
			module_section_alloc(mem_idx, ms->sects[sect_idx].sh_size, K_NO_WAIT);
		module_seek(ms, ms->sects[sect_idx].sh_offset);
		module_read(ms, m->mem[mem_idx], ms->sects[sect_idx].sh_size);

		m->mem_size += shdr.sh_size;

		LOG_DBG("Copied section %s (idx: %d, size: %d, addr %x) to mem %d, module size %d",
			name, i, shdr.sh_size, shdr.sh_addr,
			mem_idx, m->mem_size);
	}

	/* Iterate all symbols in symtab and update its st_value,
	 * for sections, using its loading address,
	 * for undef functions or variables, find it's address globally.
	 */
	elf_sym_t sym;
	size_t ent_size = ms->sects[MOD_SECT_SYMTAB].sh_entsize;
	size_t syms_size = ms->sects[MOD_SECT_SYMTAB].sh_size;
	size_t func_syms_cnt = 0;

	pos = ms->sects[MOD_SECT_SYMTAB].sh_offset;
	sym_cnt = syms_size / sizeof(elf_sym_t);

	LOG_DBG("symbol count %d", sym_cnt);

	/* Count global functions, exported by the object */
	for (i = 0; i < sym_cnt; i++, pos += ent_size) {
		if (!i)
			/* A dummy entry */
			continue;

		module_seek(ms, pos);
		module_read(ms, &sym, ent_size);

		uint32_t stt = ELF_ST_TYPE(sym.st_info);
		uint32_t stb = ELF_ST_BIND(sym.st_info);
		uint32_t sect = sym.st_shndx;

		module_seek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset + sym.st_name);
		module_read(ms, name, sizeof(name));

		if (stt == STT_FUNC && stb == STB_GLOBAL) {
			LOG_DBG("function symbol %d, name %s, type tag %d, bind %d, sect %d",
				i, name, stt, stb, sect);
			func_syms_cnt++;
		} else {
			LOG_DBG("unhandled symbol %d, name %s, type tag %d, bind %d, sect %d",
				i, name, stt, stb, sect);
		}
	}

	/* Copy over global function symbols to symtab */

	m->sym_tab.syms = k_heap_alloc(&module_heap, func_syms_cnt * sizeof(struct module_symbol),
				       K_NO_WAIT);
	m->sym_tab.sym_cnt = func_syms_cnt;
	pos = ms->sects[MOD_SECT_SYMTAB].sh_offset;
	j = 0;
	for (i = 0; i < sym_cnt; i++, pos += ent_size) {
		if (!i)
			/* A dummy entry */
			continue;

		module_seek(ms, pos);
		module_read(ms, &sym, ent_size);

		uint32_t stt = ELF_ST_TYPE(sym.st_info);
		uint32_t stb = ELF_ST_BIND(sym.st_info);
		uint32_t sect = sym.st_shndx;

		module_seek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset + sym.st_name);
		module_read(ms, name, sizeof(name));

		if (stt == STT_FUNC && stb == STB_GLOBAL && sect != SHN_UNDEF) {
			strncpy(m->sym_tab.syms[j].name, name, sizeof(m->sym_tab.syms[j].name));
			m->sym_tab.syms[j].tt = MODULE_SYMBOL_FUNC;

			m->sym_tab.syms[j].addr =
				(void *)((uintptr_t)m->mem[ms->sect_map[sym.st_shndx]]
					 + sym.st_value);
			LOG_DBG("function symbol %d, name %s, type %d, addr %p",
				j, name, MODULE_SYMBOL_FUNC, m->sym_tab.syms[j].addr);
			j++;
		}
	}

	/* relocations */
	pos = ms->hdr.e_shoff;

	for (i = 0; i < ms->hdr.e_shnum - 1; i++) {
		module_seek(ms, pos);
		module_read(ms, &shdr, sizeof(elf_shdr_t));

		pos += ms->hdr.e_shentsize;

		/* find relocation sections */
		if (shdr.sh_type != SHT_REL && shdr.sh_type != SHT_RELA)
			continue;

		rel_cnt = shdr.sh_size / sizeof(elf_rel_t);

		uintptr_t loc;

		module_seek(ms, ms->sects[MOD_SECT_SHSTRTAB].sh_offset + shdr.sh_name);
		module_read(ms, name, sizeof(name));

		if (strncmp(name, ".rel.text", sizeof(name)) == 0 ||
		    strncmp(name, ".rela.text", sizeof(name)) == 0) {
			loc = (uintptr_t)m->mem[MOD_MEM_TEXT];
		} else if (strncmp(name, ".rel.bss", sizeof(name)) == 0 ||
			   strncmp(name, ".rela.bss", sizeof(name)) == 0) {
			loc = (uintptr_t)m->mem[MOD_MEM_BSS];
		} else if (strncmp(name, ".rel.rodata", sizeof(name)) == 0 ||
			   strncmp(name, ".rela.rodata", sizeof(name)) == 0) {
			loc = (uintptr_t)m->mem[MOD_MEM_RODATA];
		} else if (strncmp(name, ".rel.data", sizeof(name)) == 0 ||
			   strncmp(name, ".rela.data", sizeof(name)) == 0) {
			loc = (uintptr_t)m->mem[MOD_MEM_DATA];
		} else if (strncmp(name, ".rela.plt", sizeof(name)) == 0 ||
			   strncmp(name, ".rela.dyn", sizeof(name)) == 0) {
			module_link_plt(ms, m, &shdr);
			continue;
		} else {
			continue;
		}

		LOG_DBG("relocation section %s (%d) linked to section %d has %d relocations",
			name, i, shdr.sh_link, rel_cnt);

		for (j = 0; j < rel_cnt; j++) {
			/* get each relocation entry */
			module_seek(ms, shdr.sh_offset + j * sizeof(elf_rel_t));
			module_read(ms, &rel, sizeof(elf_rel_t));

			/* get corresponding symbol */
			module_seek(ms, ms->sects[MOD_SECT_SYMTAB].sh_offset
				    + ELF_R_SYM(rel.r_info) * sizeof(elf_sym_t));
			module_read(ms, &sym, sizeof(elf_sym_t));

			module_seek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset +
				    sym.st_name);
			module_read(ms, name, sizeof(name));

			LOG_DBG("relocation %d:%d info %x (type %d, sym %d) offset %d sym_name %s sym_type %d sym_bind %d sym_ndx %d",
				i, j, rel.r_info, ELF_R_TYPE(rel.r_info), ELF_R_SYM(rel.r_info),
				rel.r_offset, name, ELF_ST_TYPE(sym.st_info),
				ELF_ST_BIND(sym.st_info), sym.st_shndx);

			uintptr_t link_addr, op_loc, op_code;

			/* If symbol is undefined, then we need to look it up */
			if (sym.st_shndx == SHN_UNDEF) {
				link_addr = (uintptr_t)module_find_sym(&SYMTAB, name);

				if (link_addr == 0) {
					LOG_ERR("Undefined symbol with no entry in symbol table %s, offset %d, link section %d",
						name, rel.r_offset, shdr.sh_link);
					/* TODO cleanup and leave */
					continue;
				} else {
					op_code = (uintptr_t)(loc + rel.r_offset);

					LOG_INF("found symbol %s at 0x%lx, updating op code 0x%lx",
						name, link_addr, op_code);
				}
			} else if (ELF_ST_TYPE(sym.st_info) == STT_SECTION) {
				link_addr = (uintptr_t)m->mem[ms->sect_map[sym.st_shndx]];

				LOG_INF("found section symbol %s addr 0x%lx", name, link_addr);
			}

			op_loc = loc + rel.r_offset;

			LOG_INF("relocating (linking) symbol %s type %d binding %d ndx %d offset %d link section %d",
				name, ELF_ST_TYPE(sym.st_info), ELF_ST_BIND(sym.st_info),
				sym.st_shndx, rel.r_offset, shdr.sh_link);

			LOG_INF("writing relocation symbol %s type %d sym %d at addr 0x%lx addr 0x%lx",
				name, ELF_R_TYPE(rel.r_info), ELF_R_SYM(rel.r_info),
				op_loc, link_addr);

			/* relocation */
			arch_elf_relocate(&rel, op_loc, link_addr);
		}
	}

	LOG_DBG("loaded module, .text at %p, .rodata at %p",
		m->mem[MOD_MEM_TEXT], m->mem[MOD_MEM_RODATA]);
	return 0;
}

int module_load(struct module_stream *ms, const char *name, struct module **m)
{
	int ret = 0;
	elf_ehdr_t ehdr;

	if (!SYMTAB.sym_cnt) {
		SYMTAB.sym_cnt = &_exp_sym_end - &_exp_sym_start;
		SYMTAB.syms = &_exp_sym_start;
	}

	module_seek(ms, 0);
	module_read(ms, &ehdr, sizeof(ehdr));

	/* check whether this is an valid elf file */
	if (memcmp(ehdr.e_ident, ELF_MAGIC, sizeof(ELF_MAGIC)) != 0) {
		LOG_HEXDUMP_ERR(ehdr.e_ident, 16, "Invalid ELF, magic does not match");
		return -EINVAL;
	}

	switch (ehdr.e_type) {
	case ET_REL:
	case ET_DYN:
		LOG_DBG("Loading relocatable or shared elf");
		*m = k_heap_alloc(&module_heap, sizeof(struct module), K_NO_WAIT);

		for (int i = 0; i < MOD_MEM_COUNT; i++) {
			(*m)->mem[i] = NULL;
		}

		if (m == NULL) {
			LOG_ERR("Not enough memory for module metadata");
			ret = -ENOMEM;
		} else {
			ms->hdr = ehdr;
			ret = module_load_rel(ms, *m);
		}
		break;
	default:
		LOG_ERR("Unsupported elf file type %x", ehdr.e_type);
		/* unsupported ELF file type */
		*m = NULL;
	}

	if (ret != 0) {
		if (*m != NULL) {
			module_unload(*m);
		}
		*m = NULL;
	} else {
		strncpy((*m)->name, name, sizeof((*m)->name));
		sys_slist_append(&_module_list, &(*m)->_mod_list);
	}

	return ret;
}

void module_unload(struct module *m)
{
	__ASSERT(m, "Expected non-null module");

	sys_slist_find_and_remove(&_module_list, &m->_mod_list);

	for (int i = 0; i < MOD_MEM_COUNT; i++) {
		if (m->mem[i] != NULL) {
			LOG_DBG("freeing memory region %d", i);
			k_heap_free(&module_heap, m->mem[i]);
			m->mem[i] = NULL;
		}
	}

	if (m->sym_tab.syms != NULL) {
		LOG_DBG("freeing symbol table");
		k_heap_free(&module_heap, m->sym_tab.syms);
		m->sym_tab.syms = NULL;
	}

	LOG_DBG("freeing module");
	k_heap_free(&module_heap, (void *)m);
}

int module_call_fn(struct module *m, const char *sym_name)
{
	void (*fn)(void) = module_find_sym(&m->sym_tab, sym_name);

	if (fn == NULL) {
		return -EINVAL;
	}

	fn();

	return 0;
}
