/* Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Simple driver over a WM8904 codec that provides an rtio iodev
 *
 * Very much like the I2S interface but includes the Codec configuration.
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

/**
 * Stream wrapper type for WM8904
 */
struct wm8904 {
	struct rtio_iodev iodev;
	const struct device *dev;
};

/**
 * Analog output config
 */
union wm8904_analog_out_cfg {
	uint16_t full;
	struct wm8904_analog_out_bits {
		uint8_t _unused : 7;
		bool mute : 1;	    /* Mute if true */
		bool lr_update : 1; /* Update both left and right simultaneously
				     */
		bool left_zc_en : 1; /* Zero-cross enable */
		/* 6 bit volume mapping from -57dB (at 0x00) to +6dB (at 0x3F)
		 */
		uint8_t volume : 6;
	} bits __packed;
};

/**
 * Simple configuration stuct, could grow enormous to cover all the WM8904
 * Features
 */
struct wm8904_cfg {
	union wm8904_analog_out_cfg headphone_left;
	union wm8904_analog_out_cfg headphone_right;
};

/**
 * Configure a WM8904 Codec
 *
 * @param dev WM8904 Device
 * @param cfg Configuration
 * @retval 0 Success
 * @retval -ERRNO Failure with a given errno code
 */
int wm8904_configure(const struct device *dev, struct wm8904_cfg *cfg);

/* I2C Address */
#define WM8904_ADDR 0x34

/* Array of 3 uin8t_t's from a reigster addr and value pair */
#define WM8904_PAIR(reg, val) {reg,  (uint8_t)(val >> 8 & 0x00FF), (uint8_t)(val & 0x00FF)}

/*
 * Register Map
 *
 * This is a pretty large register map and not everything is going to be used
 * but it is complete in the sense of mapping all register addresses not
 * necessarily every bit field of every register.
 *
 * In fact if simply using as an output device (in this case) the write
 * sequencer can be used to setup a common configuration in a few register
 * writes.
 */
#define WM8904_ID		 0x00
#define WM8904_BIAS_CTL0	 0x04
#define WM8904_VMID_CTL0	 0x05
#define WM8904_MIC_BIAS_CTL0	 0x06
#define WM8904_MIC_BIAS_CTL1	 0x07
#define WM8904_ANALOG_ADC0	 0x0A
#define WM8904_PWR_MGMT0	 0x0C
#define WM8904_PWR_MGMT2	 0x0E
#define WM8904_PWR_MGMT3	 0x0F
#define WM8904_PWR_MGMT6	 0x12
#define WM8904_CLK_RATES0	 0x14
#define WM8904_CLK_RATES1	 0x15
#define WM8904_CLK_RATES2	 0x16
#define WM8904_AUDIO_INTF0	 0x18
#define WM8904_AUDIO_INTF1	 0x19
#define WM8904_AUDIO_INTF2	 0x1A
#define WM8904_AUDIO_INTF3	 0x1B
#define WM8904_DAC_DVOL_LEFT	 0x1E
#define WM8904_DAC_DVOL_RIGHT	 0x1F
#define WM8904_DAC_DIGITAL0	 0x20
#define WM8904_DAC_DIGITAL1	 0x21
#define WM8904_ADC_DVOL_LEFT	 0x24
#define WM8904_ADC_DVOL_RIGHT	 0x25
#define WM8904_ADC_DIGITAL0	 0x26
#define WM8904_DMIC0		 0x27
#define WM8904_DRC0		 0x28
#define WM8904_DRC1		 0x29
#define WM8904_DRC2		 0x2A
#define WM8904_DRC3		 0x2B
#define WM8904_ANALOG_LEFT_IN0	 0x2C
#define WM8904_ANALOG_RIGHT_IN0	 0x2D
#define WM8904_ANALOG_LEFT_IN1	 0x2E
#define WM8904_ANALOG_RIGHT_IN1	 0x2F
#define WM8904_ANALOG_LEFT_OUT1	 0x39
#define WM8904_ANALOG_RIGHT_OUT1 0x3A
#define WM8904_ANALOG_LEFT_OUT2	 0x3B
#define WM8904_ANALOG_RIGHT_OUT2 0x3C
#define WM8904_ANALOG_OUT12_ZC	 0x3D
#define WM8904_DC_SERVO0	 0x43
#define WM8904_DC_SERVO1	 0x44
#define WM8904_DC_SERVO2	 0x45
#define WM8904_DC_SERVO4	 0x47
#define WM8904_DC_SERVO5	 0x48
#define WM8904_DC_SERVO6	 0x49
#define WM8904_DC_SERVO7	 0x4A
#define WM8904_DC_SERVO8	 0x4B
#define WM8904_DC_SERVO9	 0x4C
#define WM8904_DC_SERVO_RB0	 0x4D
#define WM8904_DC_ANALOG_HP0	 0x5A
#define WM8904_ANALOG_LINEOUT0	 0x5E
#define WM8904_CHARGE_PUMP0	 0x62
#define WM8904_CLASSW0		 0x68
#define WM8904_WRITE_SEQ0	 0x6C
#define WM8904_WRITE_SEQ1	 0x6D
#define WM8904_WRITE_SEQ2	 0x6E
#define WM8904_WRITE_SEQ3	 0x6F
#define WM8904_WRITE_SEQ4	 0x70
#define WM8904_FLL_CTL1		 0x74
#define WM8904_FLL_CTL2		 0x75
#define WM8904_FLL_CTL3		 0x76
#define WM8904_FLL_CTL4		 0x77
#define WM8904_FLL_CTL5		 0x78
#define WM8904_GPIO1		 0x79
#define WM8904_GPIO2		 0x7A
#define WM8904_GPIO3		 0x7B
#define WM8904_GPIO4		 0x7C
#define WM8904_DIGITAL_PULLS	 0x7E
#define WM8904_INT_STATUS	 0x7F
#define WM8904_INT_STATUS_MASK	 0x80
#define WM8904_INT_POLARITY	 0x81
#define WM8904_INT_BOUNCE	 0x82
#define WM8904_EQ1		 0x86
#define WM8904_EQ2		 0x87
#define WM8904_EQ3		 0x88
#define WM8904_EQ4		 0x89
#define WM8904_EQ5		 0x8A
#define WM8904_EQ6		 0x8B
#define WM8904_EQ7		 0x8C
#define WM8904_EQ8		 0x8D
#define WM8904_EQ9		 0x8E
#define WM8904_EQ10		 0x8F
#define WM8904_EQ11		 0x90
#define WM8904_EQ12		 0x91
#define WM8904_EQ13		 0x92
#define WM8904_EQ14		 0x93
#define WM8904_EQ15		 0x94
#define WM8904_EQ16		 0x95
#define WM8904_EQ17		 0x96
#define WM8904_EQ18		 0x97
#define WM8904_EQ19		 0x98
#define WM8904_EQ20		 0x99
#define WM8904_EQ21		 0x9A
#define WM8904_EQ22		 0x9B
#define WM8904_EQ23		 0x9C
#define WM8904_EQ24		 0x9D
#define WM8904_ADC_TEST0	 0xC6
#define WM8904_FLL_NCO_TEST0	 0xF7
#define WM8904_FLL_NCO_TEST1	 0xF8
