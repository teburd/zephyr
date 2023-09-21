/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/modules/buf_stream.h>

int module_buf_read(struct module_stream *s, struct module_memkind mk,
		    void **mem,
		    size_t len)
{
	struct module_buf_stream *buf_s = CONTAINER_OF(s, struct module_buf_stream, stream);
	size_t end = MIN(buf_s->pos + len, buf_s->len);
	size_t read_len = end - buf_s->pos;

	switch (mk) {
	case MOD_MEMKIND_METADATA:
		*mem = buf_s->buf[buf_s->pos];
	case MOD_MEMKIND_RO:
	case MOD_MEMKIND_RX:
	case MOD_MEMKIND_RW:
		*mem = module_alloc(s->alloc, mk, len);
		if (*mem == NULL) {
			return -ENOMEM;
		}
		memcpy(*mem, buf_s->buf + buf_s->pos, read_len);
	}

	buf_s->pos = end;

	return read_len;
}

int module_buf_seek(struct module_stream *s, size_t pos)
{
	struct module_buf_stream *buf_s = CONTAINER_OF(s, struct module_buf_stream, stream);

	buf_s->pos = MIN(pos, buf_s->len);

	return 0;
}
