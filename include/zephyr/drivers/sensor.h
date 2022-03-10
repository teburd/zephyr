/**
 * @file drivers/sensor.h
 *
 * @brief Public APIs for the sensor driver.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_

#ifdef CONFIG_SENSOR_VERSION_2
#include "drivers/sensor_v2.h"
#else
#include "drivers/sensor_v1.h"
#endif
#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_ */
