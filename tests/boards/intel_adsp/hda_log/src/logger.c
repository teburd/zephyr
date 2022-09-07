/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log_backend_adsp_hda.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/ztest.h>
#include <intel_adsp_ipc.h>
#include "tests.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hda_test, LOG_LEVEL_DBG);



static struct k_timer some_timer;

static int x = 0;
static int last_mod;
void periodic_work(struct k_timer *tm) 
{
	x++;
	last_mod = x%100;
	
	if (last_mod == 0 ) {
		printk("last mod %u\n", last_mod);
	}
}

ZTEST(intel_adsp_hda_log, test_hda_logger)
{
	TC_PRINT("Testing hda log backend\n");


	k_timer_init(&some_timer, periodic_work, NULL);
	k_timer_start(&some_timer,
		      K_MSEC(1),
		      K_MSEC(1));	/* Wait a moment so the output isn't mangled */

	k_msleep(100);

	/* Ensure multiple wraps and many many logs are written without delays */
	for (int i = 0; i < 16384; i++) {
		if (IS_ENABLED(CONFIG_LOG_PRINTK)) {
			printk("test hda log message %d\n", i);
		} else {
			LOG_INF("test hda log message %d", i);
		}
	}
}

ZTEST_SUITE(intel_adsp_hda_log, NULL, NULL, NULL, NULL, NULL);
