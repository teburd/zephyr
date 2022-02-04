/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_TESTS_INTEL_ADSP_TESTS_H
#define ZEPHYR_TESTS_INTEL_ADSP_TESTS_H

#include <cavs_ipc.h>
#include <cavs_test.h>

void test_post_boot_ipi(void);
void test_smp_boot_delay(void);
void test_host_ipc(void);
void test_cpu_behavior(void);
void test_cpu_halt(void);

#endif /* ZEPHYR_TESTS_INTEL_ADSP_TESTS_H */
