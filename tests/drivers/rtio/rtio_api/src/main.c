/*
 * Copyright (c) 2019 Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_rtio
 * @{
 * @defgroup t_rtio_basic_api test_rtio_basic_api
 * @}
 */

#include <ztest.h>

extern void test_rtio_pool(void);
extern void test_rtio_pipe(void);

void test_main(void)
{
	ztest_test_suite(rtio_basic_api_test,
			 ztest_unit_test(test_rtio_pool),
			 ztest_unit_test(test_rtio_pipe)
			 );
	ztest_run_test_suite(rtio_basic_api_test);
}
