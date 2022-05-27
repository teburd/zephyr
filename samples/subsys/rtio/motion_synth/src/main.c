/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "device.h"
#include "lis2mdl.h"
#include "lsm6dso.h"
#include "wm8904.h"
#include "wm8960.h"

#include <math.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/rtio_executor_concurrent.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mosyn, LOG_LEVEL_DBG);

#define SAMPLE_NO 32

/* The data_l represent a sine wave */
static int16_t data_l[SAMPLE_NO] = {
	6392,	12539,	18204,	23169,	27244,	30272,	32137,	32767,	32137,	30272,	27244,
	23169,	18204,	12539,	6392,	0,	-6393,	-12540, -18205, -23170, -27245, -30273,
	-32138, -32767, -32138, -30273, -27245, -23170, -18205, -12540, -6393,	-1,
};

/* The data_r represent a sine wave with double the frequency of data_l */
static int16_t data_r[SAMPLE_NO] = {
	12539,	23169,	30272,	32767,	30272,	23169,	12539,	0,	-12540, -23170, -30273,
	-32767, -30273, -23170, -12540, -1,	12539,	23169,	30272,	32767,	30272,	23169,
	12539,	0,	-12540, -23170, -30273, -32767, -30273, -23170, -12540, -1,
};

#define FRAME_CLK_FREQ 44100
#define TIMEOUT	       10
#define BLOCK_SIZE     (2 * sizeof(data_l))
#define NUM_TX_BLOCKS  4


RTIO_EXECUTOR_CONCURRENT_DEFINE(mosyn_exec, 3);
RTIO_DEFINE(mosyn_io, (struct rtio_executor *)(&mosyn_exec), 8, 8);

/**
 * Synthesize a sine wave sample
 *
 * @param sample_rate Sample rate in Hz
 * @param time Time step to generate for (where each step is 1/sample_rate)
 * @param freq Frequency of the sine wave in Hz
 * @param phase Phase of the sine wave in degrees
 * @return scaled estimated sine value
 */
float sine_synth(float sample_rate, uint32_t time, float freq, float phase)
{
	return sinf(time * (freq / sample_rate) + phase);
}

int main()
{
	LOG_INF("Motion Synthesizer!\n");

	int ret;
	int16_t tx_block[SAMPLE_NO];
	struct rtio_sqe *sqe;
	struct rtio_cqe *cqe;

	memset(tx_block, 0, SAMPLE_NO);
	
	const struct device *codec= DEVICE_DT_GET_ONE(wolfson_wm8960);
	struct rtio_wm8960 wm8960;
	
	ret = wm8960_get_iodev(codec, &wm8960, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Error getting codec tx stream");
		return ret;
	}

	sqe = rtio_spsc_acquire(mosyn_io.sq);
	__ASSERT(sqe != NULL, "expected a valid pointer to a sqe");
	
	rtio_sqe_prep_write(sqe, wm8960.iodev, RTIO_PRIO_HIGH, (void*)tx_block, sizeof(tx_block), tx_block);	
	
	while (true) {	
		sqe = rtio_spsc_acquire(mosyn_io.sq);
		__ASSERT(sqe != NULL, "expected a valid pointer to a sqe");
		rtio_sqe_prep_write(sqe, wm8960.iodev, RTIO_PRIO_HIGH, (void*)tx_block, sizeof(tx_block), tx_block);	
		
		ret = rtio_submit(&mosyn_io, 1);
		if (ret != 0) {
			LOG_WRN("failed to submit requests to audio stream, under/over run likely");
			goto out;
		}

		cqe = rtio_cqe_consume(&mosyn_io);
		if (cqe->result != 0) {
			LOG_WRN("failed to write buffer to audio stream, under/over run likely");
			goto out;
		}
	}
	
out:
	return 0;
}
