/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include <zephyr/modules/elf.h>
#include <zephyr/modules/module.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modules, CONFIG_MODULES_LOG_LEVEL);

#include <string.h>

static struct module_symtable SYMTAB;

static const char ELF_MAGIC[] = {0x7f, 'E', 'L', 'F'};

int module_read(struct module_stream *s, enum module_memkind mk,
		void **mem, size_t len)
{
	return s->read(s, mk, mem, len);
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

void *module_find_sym(const struct module_symtable *sym_table, const char *sym_name)
{
	elf_word i;

	/* find symbols in module */
	for (i = 0; i < sym_table->sym_cnt; i++) {
		if (strcmp(sym_table->syms[i].name, sym_name) == 0) {
			return sym_table->syms[i].addr;
		}
	}

	return NULL;
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
	elf_ehdr_t *ehdr;
	elf_word i, j, sym_cnt, rel_cnt;
	elf_rel_t rel;
	char name[32];

	m->mem_size = 0;
	m->sym_tab.sym_cnt = 0;

	module_seek(ms, 0);
	module_read(ms, MOD_MEMKIND_METADATA, &ehdr, sizeof(elf_ehdr_t));
	ms->hdr = ehdr;

	elf_shdr_t *shdr;
	size_t pos = ehdr.e_shoff;

	ms->sect_map = k_heap_alloc(&module_heap, ehdr.e_shnum*sizeof(uint32_t), K_NO_WAIT);
	ms->sect_cnt = ehdr.e_shnum;

	/* Find string tables */
	for (i = 0; i < ehdr.e_shnum; i++) {
		module_seek(ms, pos);
		module_read(ms, (void *)&shdr, sizeof(elf_shdr_t));

		pos += ehdr.e_shentsize;

		LOG_DBG("section %d at %x: name %d, type %d, flags %x, addr %x, size %d",
			i,
			ehdr.e_shoff+i*ehdr.e_shentsize,
			shdr.sh_name,
			shdr.sh_type,
			shdr.sh_flags,
			shdr.sh_addr,
			shdr.sh_size);

		switch (shdr.sh_type) {
		case SHT_SYMTAB:
			LOG_DBG("symtab at %d", i);
			ms->sects[MOD_SECT_SYMTAB] = shdr;
			ms->sect_map[i] = MOD_SECT_SYMTAB;
			break;
		case SHT_STRTAB:
			if (ehdr.e_shstrndx == i) {
				LOG_DBG("shstrtab at %d", i);
				ms->sects[MOD_SECT_SHSTRTAB] = shdr;
				ms->sect_map[i] = MOD_SECT_SHSTRTAB;
			} else {
				LOG_DBG("strtab at %d", i);
				ms->sects[MOD_SECT_STRTAB] = shdr;
				ms->sect_map[i] = MOD_SECT_STRTAB;
				break;
			}
			break;
		default:
			break;
		}
	}

	pos = ehdr.e_shoff;

	/* Copy over useful sections */
	for (i = 0; i < ehdr.e_shnum; i++) {
		module_seek(ms, pos);
		module_read(ms, (void *)&shdr, sizeof(elf_shdr_t));

		pos += ehdr.e_shentsize;

		elf32_word str_idx = shdr.sh_name;

		module_seek(ms, ms->sects[MOD_SECT_SHSTRTAB].sh_offset + str_idx);
		module_read(ms, name, sizeof(name));

		LOG_DBG("section %d name %s", i, name);

		bool valid = true;
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
		} else {
			LOG_DBG("Not copied section %s", name);
			valid = false;
		}

		if (valid) {
			ms->sects[sect_idx] = shdr;
			ms->sect_map[i] = sect_idx;

			m->mem[mem_idx] =
				module_mem_alloc(mem_idx, ms->sects[sect_idx].sh_size);
			module_seek(ms, ms->sects[sect_idx].sh_offset);
			module_read(ms, m->mem[mem_idx], ms->sects[sect_idx].sh_size);

			m->mem_size += ms->sects[sect_idx].sh_size;

			LOG_DBG("Copied section %s (idx: %d, size: %d, addr %x)"
				" to mem %d, module size %d", name, i,
				ms->sects[sect_idx].sh_size,
				ms->sects[sect_idx].sh_addr,
				mem_idx,
				m->mem_size);
		}
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

	for (i = 0; i < sym_cnt; i++) {
		module_seek(ms, pos);
		module_read(ms, &sym, ent_size);
		pos += ent_size;

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

	m->sym_tab.syms = k_heap_alloc(&module_heap, func_syms_cnt*sizeof(struct module_symbol),
				       K_NO_WAIT);
	m->sym_tab.sym_cnt = func_syms_cnt;
	pos = ms->sects[MOD_SECT_SYMTAB].sh_offset;
	j = 0;
	for (i = 0; i < sym_cnt; i++) {
		module_seek(ms, pos);
		module_read(ms, &sym, ent_size);
		pos += ent_size;

		uint32_t stt = ELF_ST_TYPE(sym.st_info);
		uint32_t stb = ELF_ST_BIND(sym.st_info);
		uint32_t sect = sym.st_shndx;

		module_seek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset + sym.st_name);
		module_read(ms, name, sizeof(name));

		if (stt == STT_FUNC && stb == STB_GLOBAL && sect != SHN_UNDEF) {
			size_t name_sz = sizeof(name);

			m->sym_tab.syms[j].name = module_alloc_ro(sizeof(name));
			strncpy(m->sym_tab.syms[j].name, name, name_sz);
			m->sym_tab.syms[j].addr =
				(void *)((uintptr_t)m->mem[ms->sect_map[sym.st_shndx]]
					 + sym.st_value);
			LOG_DBG("function symbol %d name %s addr %p",
				j, name, m->sym_tab.syms[j].addr);
			j++;
		}
	}

	/* relocations */
	pos = ehdr.e_shoff;

	for (i = 0; i < ehdr.e_shnum - 1; i++) {
		module_seek(ms, pos);
		module_read(ms, (void *)&shdr, sizeof(elf_shdr_t));

		pos += ehdr.e_shentsize;

		/* find relocation sections */
		if ((shdr.sh_type == SHT_REL)
				|| (shdr.sh_type == SHT_RELA)) {
			rel_cnt = shdr.sh_size / sizeof(elf_rel_t);

			uintptr_t loc = 0;

			module_seek(ms, ms->sects[MOD_SECT_SHSTRTAB].sh_offset + shdr.sh_name);
			module_read(ms, name, sizeof(name));

			if (strncmp(name, ".rel.text", sizeof(name)) == 0) {
				loc = (uintptr_t)m->mem[MOD_MEM_TEXT];
			} else if (strncmp(name, ".rela.text", sizeof(name)) == 0) {
				loc = (uintptr_t)m->mem[MOD_MEM_TEXT];
			} else if (strncmp(name, ".rel.bss", sizeof(name)) == 0) {
				loc = (uintptr_t)m->mem[MOD_MEM_BSS];
			} else if (strncmp(name, ".rel.rodata", sizeof(name)) == 0) {
				loc = (uintptr_t)m->mem[MOD_MEM_RODATA];
			} else if (strncmp(name, ".rel.data", sizeof(name)) == 0) {
				loc = (uintptr_t)m->mem[MOD_MEM_DATA];
			}


			LOG_DBG("relocation section %s (%d) linked to section %d has %d relocations", name, i, shdr.sh_link, rel_cnt);

			for (j = 0; j < rel_cnt; j++) {
				/* get each relocation entry */
				module_seek(ms, shdr.sh_offset
						+ j * sizeof(elf_rel_t));
				module_read(ms, (void *)&rel, sizeof(elf_rel_t));

				/* get corresponding symbol */
				module_seek(ms, ms->sects[MOD_SECT_SYMTAB].sh_offset
						+ ELF_R_SYM(rel.r_info)
						* sizeof(elf_sym_t));
				module_read(ms, &sym, sizeof(elf_sym_t));

				module_seek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset +
							sym.st_name);
				module_read(ms, name, sizeof(name));

				LOG_DBG("relocation %d:%d info %x (type %d, sym %d) offset %d sym_name %s sym_type %d sym_bind %d sym_ndx %d",
					i, j, rel.r_info, ELF_R_TYPE(rel.r_info), ELF_R_SYM(rel.r_info),
					rel.r_offset, name, ELF_ST_TYPE(sym.st_info), ELF_ST_BIND(sym.st_info), sym.st_shndx);

				uintptr_t link_addr = 0, op_loc = 0, op_code = 0;

				/* If symbol is undefined, then we need to look it up */
				if (sym.st_shndx == SHN_UNDEF) {

					link_addr = (uintptr_t)(module_find_sym((struct module_symtable *)(&SYMTAB), name));

					if (link_addr == 0) {
						LOG_ERR("Undefined symbol with no entry in symbol table %s, offset %d, link section %d",
							name, rel.r_offset, shdr.sh_link);
						/* TODO cleanup and leave */
						continue;
					} else {
						op_code = (uintptr_t)(loc + rel.r_offset);

						LOG_INF("found symbol %s at 0x%lx, updating op code 0x%lx", name, link_addr, op_code);
					}

				} else if (ELF_ST_TYPE(sym.st_info) == STT_SECTION) {
					link_addr = (uintptr_t)m->mem[ms->sect_map[sym.st_shndx]];

					LOG_INF("found section symbol %s addr 0x%lx", name, link_addr);
				}

				op_loc = loc + rel.r_offset;

				LOG_INF("relocating (linking) symbol %s type %d binding %d ndx %d offset %d link section %d",
					name, ELF_ST_TYPE(sym.st_info), ELF_ST_BIND(sym.st_info), sym.st_shndx,
					rel.r_offset, shdr.sh_link);

				LOG_INF("writing relocation symbol %s type %d sym %d at addr 0x%lx addr 0x%lx",
					name, ELF_R_TYPE(rel.r_info), ELF_R_SYM(rel.r_info), op_loc, link_addr);

				/* relocation */
				arch_elf_relocate(&rel, op_loc, link_addr);

			}
		}
	}

	LOG_DBG("loaded module, .text at %p, .rodata at %p", m->mem[MOD_MEM_TEXT], m->mem[MOD_MEM_RODATA]);
	return 0;
}

STRUCT_SECTION_START_EXTERN(module_symbol);

int module_load(struct module_stream *ms, const char name[16], struct module *m)
{
	int ret;

	if (!SYMTAB.sym_cnt) {
		STRUCT_SECTION_COUNT(module_symbol, &SYMTAB.sym_cnt);
		SYMTAB.syms = STRUCT_SECTION_START(module_symbol);
	}

	(void)module_seek(ms, 0);
	res = module_read(ms, MOD_MEMKIND_METADATA, &ms->hdr, sizeof(struct elf_ehdr_t));

	if (res != 0) {
		LOG_ERR("Reading ELF header failed");
		return ret;
	}

	/* check whether this is an valid elf file */
	if (memcmp(ms->hdr->e_ident, ELF_MAGIC, sizeof(ELF_MAGIC)) != 0) {
		LOG_HEXDUMP_ERR(ms->hdr->e_ident, 16, "Invalid ELF, magic does not match");
		return -EINVAL;
	}

	if (ms->hdr->e_type == ET_REL) {
		LOG_DBG("Loading relocatable elf");
		ret = module_load_rel(ms, m);
	} else {
		LOG_ERR("Unsupported elf file type %x", ehdr.e_type);
	}

	if (ret == 0) {
		strncpy((*m)->name, name, sizeof((*m)->name));
		sys_slist_append(&_module_list, &(*m)->_mod_list);
	}

	module_stream_free(ms);

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
