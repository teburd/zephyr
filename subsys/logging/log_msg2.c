/*
 * Copyright (c) 2020 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdarg.h>
#include <zephyr.h>
#include <syscall_handler.h>
#include <logging/log_internal.h>
#include <logging/log_ctrl.h>
#include <logging/log_frontend.h>
#include <logging/log_backend.h>

/* Returns true if any backend is in use. */
#define BACKENDS_IN_USE() \
	!(IS_ENABLED(CONFIG_LOG_FRONTEND) && \
	 (IS_ENABLED(CONFIG_LOG_FRONTEND_ONLY) || log_backend_count_get() == 0))

void z_log_msg2_finalize(struct log_msg2 *msg, const void *source,
			 const struct log_msg2_desc desc, const void *data)
{
	if (!msg) {
		printk("z_log_dropped\n");
		z_log_dropped(false);

		return;
	}

	if (data) {
		uint8_t *d = msg->data + desc.package_len;
		printk("doing memcpy to %p from %p, len %lu\n", d, data, desc.data_len);
		memcpy(d, data, desc.data_len);
	}

	msg->hdr.desc = desc;
	msg->hdr.source = source;
	printk("commiting msg\n");
	z_log_msg2_commit(msg);
	printk("commited\n");
}

void z_impl_z_log_msg2_static_create(const void *source,
			      const struct log_msg2_desc desc,
			      uint8_t *package, const void *data)
{
	if (IS_ENABLED(CONFIG_LOG_FRONTEND)) {
		log_frontend_msg(source, desc, package, data);
	}

	if (!BACKENDS_IN_USE()) {
		return;
	}

	uint32_t msg_wlen = log_msg2_get_total_wlen(desc);
	struct log_msg2 *msg = z_log_msg2_alloc(msg_wlen);

	if (msg) {
		memcpy(msg->data, package, desc.package_len);
	}

	z_log_msg2_finalize(msg, source, desc, data);
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_z_log_msg2_static_create(const void *source,
			      const struct log_msg2_desc desc,
			      uint8_t *package, const void *data)
{
	return z_impl_z_log_msg2_static_create(source, desc, package, data);
}
#include <syscalls/z_log_msg2_static_create_mrsh.c>
#endif

void z_impl_z_log_msg2_runtime_vcreate(uint8_t domain_id, const void *source,
				uint8_t level, const void *data, size_t dlen,
				uint32_t package_flags, const char *fmt, va_list ap)
{
	int plen;

	if (fmt) {
		va_list ap2;

		va_copy(ap2, ap);
		printk("%s:%d\n", __func__, __LINE__);
		plen = cbvprintf_package(NULL, Z_LOG_MSG2_ALIGN_OFFSET,
					 package_flags, fmt, ap2);
		printk("%s:%d\n", __func__, __LINE__);

		__ASSERT_NO_MSG(plen >= 0);
		va_end(ap2);
	} else {
		plen = 0;
	}

	size_t msg_wlen = Z_LOG_MSG2_ALIGNED_WLEN(plen, dlen);
	printk("plen %d, msg_wlen %lu\n", plen, msg_wlen);
	struct log_msg2 *msg;
	uint8_t *pkg;
	struct log_msg2_desc desc =
		Z_LOG_MSG_DESC_INITIALIZER(domain_id, level, plen, dlen);

	if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED) && BACKENDS_IN_USE()) {
		msg = z_log_msg2_alloc(msg_wlen);
		if (IS_ENABLED(CONFIG_LOG_FRONTEND) && msg == NULL) {
			printk("alloca pkg %d\n", plen);
			pkg = alloca(plen);
		} else {
			pkg = msg ? msg->data : NULL;
		}
	} else {
		printk("msg alloca %lu\n", msg_wlen*sizeof(int));
		msg = alloca(msg_wlen * sizeof(int));
		pkg = msg->data;
	}
	printk("pkg %p, msg %p, msg->data %p\n", pkg, msg, msg->data);

	if (pkg && fmt) {
		va_list ap3;

		printk("%s:%d calling cbvprintf_package with ap\n", __func__, __LINE__);
		va_copy(ap3, ap);
		plen = cbvprintf_package(pkg, (size_t)plen, package_flags, fmt, ap3);
		va_end(ap3);

		__ASSERT_NO_MSG(plen >= 0);
		printk("%s:%d done\n", __func__, __LINE__);

	}

	if (IS_ENABLED(CONFIG_LOG_FRONTEND)) {
		printk("%s:%d frontend_msg\n", __func__, __LINE__);

		log_frontend_msg(source, desc, pkg, data);
		printk("%s:%d done\n", __func__, __LINE__);

	}

	if (BACKENDS_IN_USE()) {
		printk("%s:%d msg2_finalize\n", __func__, __LINE__);

		msg->hdr.fmt = fmt;
		va_copy(msg->hdr.args, ap);
		z_log_msg2_finalize(msg, source, desc, data);
		va_end(msg->hdr.args);

		printk("%s:%d done\n", __func__, __LINE__);

	}
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_z_iog_msg2_runtime_vcreate(uint8_t domain_id,
				const void *source,
				uint8_t level, const void *data, size_t dlen,
				uint32_t package_flags, const char *fmt, va_list ap)
{
	return z_impl_z_log_msg2_runtime_vcreate(domain_id, source, level, data,
						dlen, package_flags, fmt, ap);
}
#include <syscalls/z_log_msg2_runtime_vcreate_mrsh.c>
#endif
