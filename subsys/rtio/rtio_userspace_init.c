/*
 * Copyright (c) 2023 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/app_memory/mem_domain.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/toolchain/common.h>
#include <zephyr/rtio/rtio.h>

struct k_mem_domain rtio_domain;

static int rtio_sys_init(const struct device *d)
{
	ARG_UNUSED(d);

	k_mem_domain_init(&rtio_domain, 0, NULL);
	STRUCT_SECTION_FOREACH(rtio, it) {
		if (it->mempool == NULL) {
			continue;
		}
		k_mem_domain_add_partition(&rtio_domain, it->mempool_buf_partition);
		it->mempool_domain = &rtio_domain;
	}

}

SYS_INIT(rtio_sys_init, APPLICATION, 1);
