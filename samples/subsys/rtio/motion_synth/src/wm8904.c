/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wolfson_wm8904

#include "wm8904.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wm8904, LOG_LEVEL_DBG);

/**
 * Statically defined Quick Start-Up (defined in wm8904_rev4.1.pdf reference, page 131).
 * 
 * Assumes a 12.288MHz clock and enables the DAC (headphones).
 */
static const uint8_t wm8904_init_seq[][2] = {
	WM8904_PAIR(WM8904_WRITE_SEQ0, 0x0100),
	WM8904_PAIR(WM8904_WRITE_SEQ3, 0x0100),
	WM8904_PAIR(WM8904_DAC_DIGITAL1, 0x0000),
};

#define WM8904_NODE_ID wm8904

/* TODO replace i2s mem slab with rtio iodev */
K_MEM_SLAB_DEFINE(tx_mem_slab, 128, 4, 32);

struct wm8904_config {
	const struct device *i2c_dev;
	const struct device *i2s_dev;
};

static const struct wm8904_config config = {
	.i2c_dev = DEVICE_DT_GET(DT_PHANDLE(WM8904_NODE_ID, i2c_dev)),
	.i2s_dev = DEVICE_DT_GET(DT_PHANDLE(WM8904_NODE_ID, i2s_dev)),
};

struct wm8904_data {
};

static struct wm8904_data data;

/**
 * Configure WM8905 for playback with the provided output channel configurations
 */
int wm8904_configure(const struct device *dev,
		      struct wm8904_cfg *cfg)
{

	int err;
	const struct wm8904_config  *config = dev->config;
	struct wm8904_data *data = dev->data;

	for(int i = 0; i < ARRAY_SIZE(wm8904_init_seq); i++) {
		LOG_DBG("writing to reg %x value %x, raw bytes %x, %x",
			wm8904_init_seq[i][0] >> 1,
			((wm8904_init_seq[i][0] & 0x01) << 8) | wm8904_init_seq[i][1],
			wm8904_init_seq[i][0],
			wm8904_init_seq[i][1]);
		err = i2c_write(wm8904->i2c, &wm8904_init_seq[i][0], 2, WM8904_ADDR);
		__ASSERT(err == 0, "Failed to write out wm8904 init sequence");
		k_busy_wait(100);
	}

	struct i2s_config i2s_cfg;

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 48000;
	i2s_cfg.block_size = 128;
	i2s_cfg.timeout = 1000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_mem_slab;

	ret = i2s_configure(wm8904->i2s, I2S_DIR_TX, &i2s_cfg);
}


static int wm8904_init(const struct device *dev)
{
	int err;
	const struct wm8904_config *config = dev->config;
	struct wm8904_data *data = dev->data;

	
	LOG_DBG("Setting up wm8904 codec over i3c");

	for(int i = 0; i < ARRAY_SIZE(wm8904_init_seq); i++) {
		err = i2c_write(config->i2c_dev, &wm8904_init_seq[i][0], 3, WM8904_ADDR);
		__ASSERT(err == 0, "Failed to write out wm8904 init sequence");
	}

	/* TODO verify  WSEQ_BUSY set in WRITE_SEQ4 */

	k_msleep(300);

	/* TODO poll on WSEQ_BUSY in WRITE_SEQ4 for completion */

	LOG_DBG("wm8904 setup, configuring i2s");

	/* Setup i2s clocking for external 12.288 MHz clock */
	struct i2s_config i2s_cfg;
	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 48000;
	i2s_cfg.block_size = 128;
	i2s_cfg.timeout = 1000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_mem_slab;

	err = i2s_configure(config->i2s_dev, I2S_DIR_TX, &i2s_cfg);	
	__ASSERT(err == 0, "Failed to initialize i2s and clock");
	
	return err;
}

DEVICE_DT_DEFINE(DT_NODELABEL(WM8904_NODE_ID), wm8904_init, NULL, &data, &config, APPLICATION, 0, NULL)
