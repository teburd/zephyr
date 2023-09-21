/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULE_H
#define ZEPHYR_MODULE_H

#include <zephyr/sys/slist.h>
#include <zephyr/modules/elf.h>
#include <zephyr/modules/symbol.h>
#include <zephyr/modules/allocator.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Loadable modules
 * @defgroup modules Modules
 * @ingroup os_services
 * @{
 */

/**
 * @brief A symbol table
 *
 * An array of symbols
 */
struct module_symtable {
	elf_word sym_cnt;
	struct module_symbol *syms;
};

/**
 * @brief Enum of module memory regions for lookup tables
 */
enum module_mem {
	MOD_MEM_TEXT,
	MOD_MEM_DATA,
	MOD_MEM_RODATA,
	MOD_MEM_BSS,

	MOD_MEM_COUNT,
};

/**
 * @brief Enum of sections for lookup tables
 */
enum module_section {
	MOD_SECT_TEXT,
	MOD_SECT_DATA,
	MOD_SECT_RODATA,
	MOD_SECT_BSS,

	MOD_SECT_REL_TEXT,
	MOD_SECT_REL_DATA,
	MOD_SECT_REL_RODATA,
	MOD_SECT_REL_BSS,

	MOD_SECT_SYMTAB,
	MOD_SECT_STRTAB,
	MOD_SECT_SHSTRTAB,

	/* Should always be last */
	MOD_SECT_COUNT,
};

/**
 * @brief Loadable code module
 */
struct module {
	sys_snode_t _mod_list;

	/** Name of the module */
	char name[16];

	/** Lookup table of module memory regions */
	void *mem[MOD_MEM_COUNT];

	/** Total size of the module memory usage */
	size_t mem_size;

	/** Exported symbols from the module, may be used in other modules */
	struct module_symtable sym_tab;

	/** Allocator used for memory */
	struct module_allocator *alloc;
};

/**
 * @brief Module loader context
 *
 * Provides a context and required functions to load and parse an elf file.
 * The interface and context is meant to support a wide range of sources of
 * ELF files from mapped memory with already page aligned ELF sections to
 * reading off of slow serially connected flash or rom memory.
 *
 * To support this the read function is expected to return a pointer. This pointer
 * is expected to be either a direct mapping to the required information or
 * a copy of it placed in memory allocated by a module memory allocator.
 *
 * A source of an ELF stream/blob
 */
struct module_stream {
	int (*read)(struct module_stream *s, enum module_memkind mk, void **mem, size_t len);
	int (*seek)(struct module_stream *s, size_t pos);
	struct module_allocator *alloc;
	elf_ehdr_t *hdr;
	elf_shdr_t *sects[MOD_SECT_COUNT];
	uint32_t *sect_map;
	uint32_t sect_cnt;
};

/**
 * @brief Read a length of bytes updating the given pointer to the read value.
 *
 * This may be a zero-copy or copying read depending on the provided allocator
 * to the module stream. When copying the memory is allocated on behalf of the
 * caller with the memory kind being taken into account.
 *
 * E.g. for reading the .text section from the elf
 *
 * int res = module_read(ms, MOD_MEMKIND_RX, &text_sect, text_sect_len);
 *
 * E.g. for reading metadata, like an elf section header
 *
 * int res = module_read(ms, MOD_MEMKIND_METADATA, &ms->sects[MOD_SECT_TEXT], text_shdr_len);
 *
 * @param ms Module stream
 * @param mk Memory kind, e.g. MOD_MEMKIND_METADATA
 * @param mem A double pointer which will be assigned the address of the read data.
 * @param len Length of data to read from the stream
 * @retval read_len Length read from the stream
 * @retval -ENOMEM Allocation failed
 * @retval -errno Underlying stream error, could be an errno
 */
int module_read(struct module_stream *ms, enum module_memkind mk, void **mem, size_t len);

/**
 * @brief Seek to an absolute location of the module (elf) file
 *
 * @param ms Module stream
 * @param pos Absolute position to seek to
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid position
 */
int module_seek(struct module_stream *ms, size_t pos);

/**
 * @brief List head of loaded modules
 */
sys_slist_t *module_list(void);

/**
 * @brief Find a module from a name
 *
 * @param name String name of the module
 * @retval NULL if no module not found
 * @retval module if module found
 */
struct module *module_from_name(const char *name);

/**
 * @brief Load a module
 *
 * Loads relevant ELF data into memory and provides a structure to work with it.
 *
 * @param[in] modstr A byte stream like object that provides a source for loadable code
 * @param[in] name A string identifier for the module
 * @param[out] module A pointer to a statically allocated module struct
 *
 * @retval 0 Success
 * @retval -ENOMEM Not enough memory
 * @retval -EINVAL Invalid ELF stream
 */
int module_load(struct module_stream *modstr, const char name[16], struct module **module);

/**
 * @brief Unload a module
 *
 * @param module Module to unload
 */
void module_unload(struct module *module);

/**
 * @brief Find the address for an arbitrary symbol name.
 *
 * @param sym_table Symbol table to lookup symbol in
 * @param sym_name Symbol name to find
 *
 * @retval NULL if no symbol found
 * @retval addr Address of symbol in memory if found
 */
void *module_find_sym(const struct module_symtable *sym_table, const char *sym_name);

/**
 * @brief Call a function by name
 *
 * Expects a symbol representing a void fn(void) style function exists
 * and may be called.
 *
 * @param module Module to call function in
 * @param sym_name Function name (exported symbol) in the module
 *
 * @retval 0 success
 * @retval -EINVAL invalid symbol name
 */
int module_call_fn(struct module *module, const char *sym_name);

/**
 * @brief Architecture specific function for updating op codes given a relocation
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this. They are instructions on how to rewrite opcodes given
 * the actual placement of some symbolic data such as a section, function,
 * or object.
 *
 * @param rel Relocation data provided by elf
 * @param opaddr Address of operation to rewrite with relocation
 * @param opval Value of looked up symbol to relocate
 */
void arch_elf_relocate(elf_rel_t *rel, uintptr_t opaddr, uintptr_t opval);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULE_H */
