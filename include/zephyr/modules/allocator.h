/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_MODULE_ALLOCATOR_H
#define ZEPHYR_MODULE_ALLOCATOR_H

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* @brief Module allocator
 * @default module_allocator
 * @ingroup modules
 * @{
 */

/**
 * @brief Enum of module memory kind that might be used for allocation
 */
enum module_memkind {
	MOD_MEMKIND_METADATA,
	MOD_MEMKIND_RX,
	MOD_MEMKIND_RO,
	MOD_MEMKIND_RW,
};

/**
 * @brief Module memory allocator
 *
 * Provides the required memory for module loading and parsing.
 */
struct module_allocator {
	void* (*alloc)(struct module_allocator *ma, enum module_memkind mk, size_t len);
	void (*free)(struct module_allocator *ma, enum module_memkind mk, void *mem);
};

/**
 * @brief Module default allocator
 *
 * Allocates memory from a single default module heap
 */
extern struct module_allocator *const module_default_allocator;

/**
 * @brief Allocate memory from the allocator for a specified usage
 *
 * @param ma Allocator
 * @param mk Memory kind
 * @param sz Size of allocation
 * @retval NULL Not enough memory to allocate from
 */
void *module_alloc(struct module_allocator *ma, enum module_memkind mk, size_t sz);


/**
 * @brief Free memory allocated from this allocator
 *
 * @param ma Allocator
 * @param mk Memory kind
 * @param mem Memory address
 */
void module_free(struct module_allocator *ma, enum module_memkind mk, void *mem);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif /* ZEPHYR_MODULE_ALLOCATOR_H */
