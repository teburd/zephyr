/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULE_BUF_STREAM_H
#define ZEPHYR_MODULE_BUF_STREAM_H

#include <zephyr/modules/module.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Module buffer stream
 * @defgroup module_buf_stream Module Buffer Stream
 * @ingroup modules
 * @{
 */

/**
 * @brief Module buffer stream
 *
 * Enables loading an elf stream directly from mapped memory. Assumes all
 * readable memory is const so can be used to load an ELF directly from
 * mapped flash or ram.
 *
 * Will not allocate any memory for metadata reads, but will still copy
 * over all relevant elf sections into RAM including .rodata.
 *
 * If custom mapping/reading strategies are needed a variation of this
 * stream should be created.
 */
struct module_buf_stream {
	struct module_stream stream;
	const uint8_t *buf;
	size_t len;
	off_t pos;
};

int module_buf_read(struct module_stream *s, void *buf, size_t len);
int module_buf_seek(struct module_stream *s, size_t pos);

#define MODULE_BUF_STREAM_INIT(_alloc, _buf, _buf_len)	\
	{						\
		.stream = {				\
			.read = module_buf_read,	\
			.seek = module_buf_seek		\
			.alloc = _alloc,		\
		},					\
		.buf = (_buf),				\
		.len = (_buf_len),			\
		.pos = 0				\
	}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULE_BUF_STREAM_H */
