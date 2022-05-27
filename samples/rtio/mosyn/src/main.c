/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "device.h"
#include "lis2mdl.h"
#include "lsm6dso.h"
#include "wm8960.h"

#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

static struct rtio_wm8960 wm8960;

/*
static struct rtio_lis2mdl lis2mdl;
static struct rtio_lsm6dso lsm6dso;
*/


static struct i2s_config i2s_cfg;

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mosyn, LOG_LEVEL_DBG);

#define SAMPLE_NO 32

/* The data_l represent a sine wave */
static int16_t data_l[SAMPLE_NO] = {
	  6392,  12539,  18204,  23169,  27244,  30272,  32137,  32767,  32137,
	 30272,  27244,  23169,  18204,  12539,   6392,      0,  -6393, -12540,
	-18205, -23170, -27245, -30273, -32138, -32767, -32138, -30273, -27245,
	-23170, -18205, -12540,  -6393,     -1,
};

/* The data_r represent a sine wave with double the frequency of data_l */
static int16_t data_r[SAMPLE_NO] = {
	 12539,  23169,  30272,  32767,  30272,  23169,  12539,      0, -12540,
	-23170, -30273, -32767, -30273, -23170, -12540,     -1,  12539,  23169,
	 30272,  32767,  30272,  23169,  12539,      0, -12540, -23170, -30273,
	-32767, -30273, -23170, -12540,     -1,
};

#define FRAME_CLK_FREQ 44100
#define TIMEOUT 10
#define BLOCK_SIZE (2 * sizeof(data_l))
#define NUM_TX_BLOCKS 4


/**
 * Fill a block with predefined data (data_r, data_l)
 * with bit shift attenuation
 */
static void fill_buf(int16_t *tx_block, int att)
{
	for (int i = 0; i < SAMPLE_NO; i++) {
		tx_block[2 * i] = data_l[i] >> att;
		tx_block[2 * i + 1] = data_r[i] >> att;
	}
}

int main()
{
	//lsm6dso_configure(&lsm6dso);
	//lis2mdl_configure(&lis2mdl);
	//wm8960_configure(&wm8960);

	LOG_INF("Motion Synthesizer!\n");

	int ret;

	const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(sai1));
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(lpi2c1));

	__ASSERT(i2c_dev != NULL, "expected a valid i2c device");
	__ASSERT(i2s_dev != NULL, "expected a valid i2s device");

	wm8960_configure(&wm8960, i2c_dev, i2s_dev);

	int16_t tx_block[SAMPLE_NO];

	/* i2s_buf_write wants to copy, so setup a buf here then copy it in */
	for (int i = 0; i < NUM_TX_BLOCKS; i++) {
		LOG_INF("filling buf");
		fill_buf((uint16_t *)tx_block, 0);
		LOG_INF("writing buf");
		i2s_buf_write(i2s_dev, tx_block, sizeof(tx_block));
	}

	LOG_INF("trigging i2s");
	i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

	while (true) {
	for (int i = 0; i < 256; i++) {
		fill_buf((uint16_t *)tx_block, 0);
		i2s_buf_write(i2s_dev, tx_block, sizeof(tx_block));
	}
	LOG_INF("wrote 128 blocks to i2s");
	}

	/* start threads for pipeline, each one pulls (reads) from the previous
	 * but the alternative is also possible where the sensor reads trigger
	 * writes (push) */
	/* k_thread_create(orientation); */
	/* k_thread_create(audiosynth); */

	/* Initialize start feeding the pipeline */
	/* rtio_prep_read(r, &lsm6dso, ...) */
	/* rtio_prep_read(r, &lis2mdl, ...) */
	/* rtio_submit(r, 0) */
	/* rtio */
}
