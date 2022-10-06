/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>

extern void power_init(void);
extern void cavs_clock_init(void);

#if CONFIG_MP_NUM_CPUS > 1
extern void soc_mp_init(void);
#endif

static __imr int soc_init(const struct device *dev)
{
	power_init();

#ifdef CONFIG_ADSP_CLOCK
	cavs_clock_init();
#endif
	/* Temporary HP clock setup */
	*((uint32_t*)(0x71B78)) = 0x207;

#if CONFIG_MP_NUM_CPUS > 1
	soc_mp_init();
#endif

	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 99);
