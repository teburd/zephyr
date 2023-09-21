/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/modules/allocator.h>
#include <zephyr/kernel.h>

/* Default heap */
K_HEAP_DEFINE(module_default_heap, CONFIG_MODULES_HEAP_SIZE * 1024);

static inline int module_mem_alloc(struct module_allocator *ma,
				   enum module_memkind mk,
				   void **mem,
				   size_t sz)
{

	void *mem = NULL;

	switch (mk) {
	case MOD_MEMKIND_METADATA:
	case MOD_MEMKIND_RO:
	case MOD_MEMKIND_RW:
	case MOD_MEMKIND_RX:
		*mem = k_heap_alloc(&module_default_heap, sz, K_NO_WAIT);
		break;
	default:
		unreacheable();
	}

	if (*mem == NULL) {
		return -ENOMEM;
	}

	return 0;
}

static inline int module_mem_free(struct module_allocator *ma, enum module_memkind mk, void *mem)
{
	k_heap_free(&module_default_heap, mem);
}

/* do not export this struct to avoid backwards compatible issues */
static struct module_allocator _module_default_allocator = {
	.alloc = module_mem_alloc,
	.free = module_mem_free,
};

struct module_allocator *module_default_allocator = &_module_default_allocator;
