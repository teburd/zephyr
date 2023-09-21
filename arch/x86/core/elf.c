/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/modules/module.h>
#include <zephyr/modules/elf.h>

/* x86 32-bit relocation for REL file */
void arch_elf_relocate(elf_rel_t *rel, uintptr_t opaddr, uintptr_t opval)
{
	elf_addr *where = (elf_addr *)(opaddr);
	elf_word reloc_type = ELF32_R_TYPE(rel->r_info);

	switch (reloc_type) {
	case R_386_32:
		*where = opval;
		break;
	case R_386_PC32:
		*where = opval - (elf_addr)where;
		break;
	default:
		/* unsupported relocation type */
		printk("Unsupported relocation type %d\n", reloc_type);
		break;
	}
}
