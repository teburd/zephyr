/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Simple driver over a LSM6DSO that provides an rtio iodev
 *
 * Ideally this uses a sensor driver that works with RTIO instead.
 */
#ifndef LSM6DSO_H_
#define LSM6DSO_H_

#include <rtio/rtio.h>

struct rtio_lsm6dso {
	struct rtio_iodev iodev;
};

void lsm6dso_configure(struct rtio_lsm6dso *iodev);

#endif /* LSM6DSO_H_ */
