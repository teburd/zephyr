/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Simple driver over a WM8960 codec that provides an rtio iodev
 *
 * Ideally this uses a codec/i2s driver that works with RTIO instead.
 */
#ifndef WM8960_H_
#define WM8960_H_

#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>

struct rtio_wm8960 {
	struct rtio_iodev iodev;
	const struct device *i2c;
	const struct device *i2s;
};

void wm8960_configure(struct rtio_wm8960 *wm8960,
		      const struct device *i2c,
		      const struct device *i2s);

#define WM8960_ADDR   0x1A
#define WM8960_VMASK  0x01FF
#define WM8960_REGSHIFT(x) (x << 1)
#define WM8960_VALH(val) ((val >> 8) && 0x01)
#define WM8960_VALL(val) (val && 0xFF)

/* Generate a pair of values for high and low bytes */
#define WM8960_PAIR(reg, val) {WM8960_REGSHIFT(reg) | WM8960_VALH(val), WM8960_VALL(val)}

#define WM8960_LIN      0x00
#define WM8960_RINL     0x01
#define WM8960_LOUT1    0x02
#define WM8960_ROUT1    0x03
#define WM8960_CLOCK1   0x04
#define WM8960_CTL1     0x05
#define WM8960_CTL2     0x06
#define WM8960_IFACE1   0x07
#define WM8960_CLOCK2   0x08
#define WM8960_IFACE2   0x09
#define WM8960_LDAC     0x0A
#define WM8960_RDAC     0x0B
#define WM8960_RESET    0x0F
#define WM8960_3DCTL    0x10
#define WM8960_ALC1     0x11
#define WM8960_ALC2     0x12
#define WM8960_ALC3     0x13
#define WM8960_NGATE    0x14
#define WM8960_LADC     0x15
#define WM8960_RADC     0x16
#define WM8960_ACTL1    0x17
#define WM8960_ACTL2    0x18
#define WM8960_PWR1     0x19
#define WM8960_PWR2     0x1A
#define WM8960_ACTL3    0x1B
#define WM8960_APOP1    0x1C
#define WM8960_APOP2    0x1D
#define WM8960_ADCL     0x20
#define WM8960_ADCR     0x21
#define WM8960_LMIX     0x22
#define WM8960_RMIX     0x25
#define WM8960_MMIX     0x27
#define WM8960_LOUT2    0x28
#define WM8960_ROUT2    0x29
#define WM8960_MONOOUT  0x2A
#define WM8960_BOOST1   0x2B
#define WM8960_BOOST2   0x2C
#define WM8960_BYPASS1  0x2D
#define WM8960_BYPASS2  0x2E
#define WM8960_PWR3     0x2F
#define WM8960_ACTL4    0x30
#define WM8960_CLASSD1  0x31
#define WM8960_CLASSD3  0x33
#define WM8960_PLLN     0x34
#define WM8960_PLLK1    0x35
#define WM8960_PLLK2    0x36
#define WM8960_PLLK3    0x37

#endif /* WM8960_H_ */
