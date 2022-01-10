/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_MODULE_BUF_STREAM_H
#define ZEPHYR_MODULE_BUF_STREAM_H

#include <zephyr/modules/module.h>

struct module_buf_stream {
	struct module_stream stream;
	const uint8_t *buf;
	size_t len;
	off_t pos;
};

int module_buf_read(struct module_stream *s, void *buf, size_t len);
int module_buf_seek(struct module_stream *s, size_t pos);

#define MODULE_BUF_STREAM(_buf, _buf_len)		\
	{						\
		.stream = {				\
			.read = module_buf_read,	\
			.seek = module_buf_seek		\
		},					\
		.buf = (_buf),				\
		.len = (_buf_len),			\
		.pos = 0				\
	}



#endif /* ZEPHYR_MODULE_BUF_STREAM_H */
