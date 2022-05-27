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

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

/**
 * A taken wm8960 device which maybe used with RTIO
 */
struct rtio_wm8960 {
	struct rtio_iodev *iodev;
	const struct device *dev;
	uint32_t key;
};

/**
 * Get ownership of the audio stream rtio_iodev from the device
 *
 * @param dev Device (codec) to get the iodev from
 * @param wm8960 An object to assign a key and iodev pointer to on success
 * @param timeout Timeout parameter to take the internal iodev (typically a mutex)
 * @retval 0 Success
 * @retval -errno Error
 */
int wm8960_get_iodev(const struct device *dev, struct rtio_wm8960 *wm8960, k_timeout_t timeout);

/**
 * Return ownership of the audio stream (rtio_wm8960) to the device
 *
 * @param dev Device (codec) to get the iodev from
 * @param wm8960 An object to return a key and iodev pointer from on success
 * @retval 0 Success
 * @retval -errno Error
 */
int wm8960_put_iodev(const struct device *dev, struct rtio_wm8960 *wm8960);

/**
 * Start streaming audio data
 *
 * @param iodev Iodev (codec) to start streaming with
 * @retval 0 Success
 * @retval -errno Error
 */
int wm8960_start(struct rtio_iodev *iodev);

/**
 * Stop streaming audio data
 *
 * @param iodev Iodev (codec) to start streaming with
 * @retval 0 Success
 * @retval -errno Error
 */
int wm8960_stop(struct rtio_iodev *iodev);

#define WM8960_VMASK	   0x01FF
#define WM8960_REGSHIFT(x) (x << 1)
#define WM8960_VALH(val)   ((val >> 8) & 0x01)
#define WM8960_VALL(val)   (val & 0xFF)

/* Generate a pair of values for high and low bytes */
#define WM8960_PAIR(reg, val)                                                                      \
	{                                                                                          \
		WM8960_REGSHIFT(reg) | WM8960_VALH(val), WM8960_VALL(val)                          \
	}

#define WM8960_LIN     0x00
#define WM8960_RIN     0x01
#define WM8960_LOUT1   0x02
#define WM8960_ROUT1   0x03
#define WM8960_CLOCK1  0x04
#define WM8960_CTL1    0x05
#define WM8960_CTL2    0x06
#define WM8960_IFACE1  0x07
#define WM8960_CLOCK2  0x08
#define WM8960_IFACE2  0x09
#define WM8960_LDAC    0x0A
#define WM8960_RDAC    0x0B
#define WM8960_RESET   0x0F
#define WM8960_3DCTL   0x10
#define WM8960_ALC1    0x11
#define WM8960_ALC2    0x12
#define WM8960_ALC3    0x13
#define WM8960_NGATE   0x14
#define WM8960_LADC    0x15
#define WM8960_RADC    0x16
#define WM8960_ACTL1   0x17
#define WM8960_ACTL2   0x18
#define WM8960_PWR1    0x19
#define WM8960_PWR2    0x1A
#define WM8960_ACTL3   0x1B
#define WM8960_APOP1   0x1C
#define WM8960_APOP2   0x1D
#define WM8960_ADCL    0x20
#define WM8960_ADCR    0x21
#define WM8960_LMIX    0x22
#define WM8960_RMIX    0x25
#define WM8960_MMIX    0x27
#define WM8960_LOUT2   0x28
#define WM8960_ROUT2   0x29
#define WM8960_MONOOUT 0x2A
#define WM8960_BOOST1  0x2B
#define WM8960_BOOST2  0x2C
#define WM8960_BYPASS1 0x2D
#define WM8960_BYPASS2 0x2E
#define WM8960_PWR3    0x2F
#define WM8960_ACTL4   0x30
#define WM8960_CLASSD1 0x31
#define WM8960_CLASSD3 0x33
#define WM8960_PLLN    0x34
#define WM8960_PLLK1   0x35
#define WM8960_PLLK2   0x36
#define WM8960_PLLK3   0x37

#endif /* WM8960_H_ */
