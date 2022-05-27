/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Simple driver over a LIS2MDL and mcux I2C that provides an RTIO IODEV
 *
 * Ideally this uses a sensor driver that works with RTIO instead.
 */
#ifndef LIS2MDL_H_
#define LIS2MDL_H_

#include <rtio/rtio.h>

struct rtio_lis2mdl {
	struct rtio_iodev iodev;
};

void lis2mdl_configure(struct rtio_lis2mdl *iodev);


#endif /* LIS2MDL_H_ */
