/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/modules/module.h>
#include <zephyr/modules/elf.h>

/**
 * @brief Architecture specific function for relocating partially linked (static) elf
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 *
 * The relocation codes for xtensa are effectively undocumented like most things
 * xtensa. However we can see what other linkers and dynamic loaders have
 * for the relocation types and updates.
 * */
void arch_elf_relocate(elf_rel_t *rel, uintptr_t opaddr, uintptr_t opval)
{
	__ASSERT(opaddr != 0, "Expects valid opcode address");

	elf_word reloc_type = ELF32_R_TYPE(rel->r_info);

	uint32_t old_opcode = *(uint32_t *)opaddr;

	switch(reloc_type) {
		case R_XTENSA_32:
			/* Update the absolute address of a literal */
			*((uint32_t*)opaddr) = (uint32_t)opval;
			break;
		case R_XTENSA_SLOT0_OP:
			/* Op-code operand update */
			*((uint32_t*)opaddr) = (uint32_t)opval;
			break;

		default:
			printk("Unsupported relocation type %d\n", reloc_type);
			break;
	}

	printk("relocate wrote addr %p from %x to %x\n", (uint32_t *)opaddr,
	       old_opcode, *(uint32_t*)opaddr);
}
