/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wolfson_wm8960

#include "wm8960.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
LOG_MODULE_REGISTER(wm8960, LOG_LEVEL_DBG);

static const uint8_t wm8960_init_seq[][2] = {
	WM8960_PAIR(WM8960_RESET, 0),	    /* Reset to defaults */
	WM8960_PAIR(WM8960_PWR1, 0x01C0),   /* vmid, vref, ain, digenb on */
	WM8960_PAIR(WM8960_PWR2, 0x01E0),   /* dac on, out1 on, spkr on, pll on */
	WM8960_PAIR(WM8960_PWR3, 0x003C),   /* mic, mix on */
	WM8960_PAIR(WM8960_IFACE1, 0x0002), /* master mode, word length 16 bits, format I2S */
	WM8960_PAIR(WM8960_IFACE2, 0x0040), /* ADC and DAC word clock same (ALRCGPIO 1) */
	WM8960_PAIR(WM8960_LDAC, 0x00FF),   /* 0dB */
	WM8960_PAIR(WM8960_RDAC, 0x01FF),   /* 0dB */
	WM8960_PAIR(WM8960_LOUT1, 0x003F),  /* 0dB */
	WM8960_PAIR(WM8960_ROUT1, 0x013F),  /* 0dB */
	WM8960_PAIR(WM8960_ADCR, 0x0144),   /* RMN1, RMP2, RMIC2B */
	WM8960_PAIR(WM8960_LMIX, 0x01F0),   /* DAC routing */
	WM8960_PAIR(WM8960_RMIX, 0x01F0),   /* DAC routing */
	WM8960_PAIR(WM8960_CTL2, 0x0004),   /* Unmute ramp volume */
	WM8960_PAIR(WM8960_CTL1, 0x0000),   /* Unmute */
	WM8960_PAIR(WM8960_CLASSD1, 0x00F7) /* enable class d */
};

#define WM8960_NODE_ID wm8960

struct wm8960_config {
	const struct i2c_dt_spec i2c;
};

struct wm8960_data {
	const struct device *i2s;
	struct k_mutex *lock;
	struct k_mem_slab *tx_mem_slab;
	struct rtio_iodev iodev;
	uint32_t iodev_key;
};

/* TODO not really asynchronous at the moment as the DMA is hidden behind a k_msgq in i2s, which isn't really
 * what I want to demonstrate.
 */
void wm8960_iodev_submit(const struct rtio_sqe *sqe, struct rtio *r)
{
	/* TODO only accept TX's, copy TX buffer to slab for now (zero copy later...) */
	if (sqe->op == RTIO_OP_TX) {
		struct wm8960_data *data = CONTAINER_OF(sqe->iodev, struct wm8960_data, iodev);

		/* TODO remove this and do a direct DMA write with the buffer */
		i2s_buf_write(data->i2s, sqe->buf, sqe->buf_len);

		/* Assume its only a TX channel here for now */
		rtio_sqe_ok(r, sqe, 0);
	}
}

const static struct rtio_iodev_api wm8960_iodev_api = {.submit = wm8960_iodev_submit};

int wm8960_get_iodev(const struct device *dev, struct rtio_wm8960 *wm8960, k_timeout_t timeout)
{
	int err;
	struct wm8960_data *data = (struct wm8960_data *)dev->data;

	err = k_mutex_lock(data->lock, timeout);
	if (err != 0) {
		LOG_WRN("Failed to obtain wm8960 mutex before timeout");
		return err;
	}

	data->iodev_key += 1;

	wm8960->key = data->iodev_key;
	wm8960->iodev = &data->iodev;
	wm8960->dev = dev;

	return 0;
}

int wm8960_put_iodev(const struct device *dev, struct rtio_wm8960 *wm8960)
{
	struct wm8960_data *data = (struct wm8960_data *)wm8960->dev->data;

	if (wm8960->key != data->iodev_key) {
		return -EINVAL;
	}

	wm8960->iodev = NULL;
	wm8960->dev = NULL;
	wm8960->key = 0;

	return k_mutex_unlock(data->lock);
}

int wm8960_start(struct rtio_iodev *iodev)
{
	struct rtio_wm8960 *wm8960 = (struct rtio_wm8960 *)iodev;
	struct wm8960_data *data = (struct wm8960_data *)(wm8960->dev->data);

	LOG_INF("Starting streaming of wm8960 %p", wm8960);

	i2s_trigger(data->i2s, I2S_DIR_TX, I2S_TRIGGER_START);

	return 0;
}

int wm8960_stop(struct rtio_iodev *iodev)
{
	struct rtio_wm8960 *wm8960 = (struct rtio_wm8960 *)iodev;
	struct wm8960_data *data = (struct wm8960_data *)(wm8960->dev->data);

	LOG_INF("Stopping streaming of wm8960 %p", wm8960);

	i2s_trigger(data->i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);

	return 0;
}

/**
 * Initialize the wm8960 codec as a simple audio output device
 */
int wm8960_init(const struct device *dev)
{
	int err;
	const struct wm8960_config *cfg = dev->config;
	struct wm8960_data *data = dev->data;

	LOG_DBG("Setting up wm8960 codec over i2c");

	for (int i = 0; i < ARRAY_SIZE(wm8960_init_seq); i++) {
		LOG_DBG("writing to reg %x value %x, raw bytes %x, %x", wm8960_init_seq[i][0] >> 1,
			((wm8960_init_seq[i][0] & 0x01) << 8) | wm8960_init_seq[i][1],
			wm8960_init_seq[i][0], wm8960_init_seq[i][1]);
		err = i2c_write_dt(&cfg->i2c, &wm8960_init_seq[i][0], 2);
		__ASSERT(err == 0, "Failed to write out wm8960 init sequence");
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
	i2s_cfg.mem_slab = data->tx_mem_slab;

	err = i2s_configure(data->i2s, I2S_DIR_TX, &i2s_cfg);

	return err;
}

#define WM8960_INIT(n)                                                                             \
	static const struct wm8960_config wm8960_config_##n = {                                    \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
                                                                                                   \
	K_MEM_SLAB_DEFINE(wm8960_tx_mem_slab_##n, 128, 4, 32);                                     \
	K_MUTEX_DEFINE(wm8960_mutex_##n);                                                          \
                                                                                                   \
	static struct wm8960_data wm8960_data_##n = {                                              \
		.lock = &wm8960_mutex_##n,                                                         \
		.tx_mem_slab = &wm8960_tx_mem_slab_##n,                                            \
		.i2s = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), i2s)),                             \
		.iodev =                                                                           \
			{                                                                          \
				.api = &wm8960_iodev_api,                                          \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, wm8960_init, NULL, &wm8960_data_##n, &wm8960_config_##n,          \
			      APPLICATION, 0, NULL);

DT_INST_FOREACH_STATUS_OKAY(WM8960_INIT);
