/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <ztest.h>
#include <cavs_ipc.h>
#include <cavs_hda.h>
#include "tests.h"

#define IPC_TIMEOUT K_MSEC(100)
#define STREAM_ID 3U
#define FIFO_SIZE 256
#define TRANSFER_SIZE 256
#define TRANSFER_COUNT 100

__attribute__((section(".dma_buffers"), aligned(128))) uint8_t hda_fifo[FIFO_SIZE];


static volatile int msg_cnt;
static volatile int msg_res;

static bool ipc_message(const struct device *dev, void *arg,
			uint32_t data, uint32_t ext_data)
{
	printk("HDA message received, data %u, ext_data %u\n", data, ext_data);
	msg_res = data;
	msg_cnt++;
	return true;
}

static void ipc_done(const struct device *dev, void *arg)
{
}


/**
 * Tests host input streams
 *
 * Note that the order of operations in this test are *extremely* important.
 * Configuring the host side buffers before the dsp side seems to cause things to become invalid.
 */
void test_hda_host_in_smoke(void)
{
	printk("smoke testing hda with fifo buffer at address %p, size %d\n", hda_fifo, FIFO_SIZE);

	cavs_ipc_set_message_handler(CAVS_HOST_DEV, ipc_message, NULL);
	cavs_ipc_set_done_handler(CAVS_HOST_DEV, ipc_done, NULL);

	struct cavs_hda_streams *host_in = &cavs_hda.host_in;

	printk("Using buffer of size %d at addr %p\n", FIFO_SIZE, hda_fifo);

	/* setup a ramp in the buffer */
	for(uint32_t i = 0; i < FIFO_SIZE; i++) {
		hda_fifo[i] = i & 0xff;
	}
	/* The buffer is in the cached address range and must be flushed when done writing. */
	z_xtensa_cache_flush(hda_fifo, FIFO_SIZE);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_RESET, STREAM_ID, IPC_TIMEOUT));
	printk("host reset: "); cavs_hda_dbg(host_in, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_CONFIG, STREAM_ID | (FIFO_SIZE << 8), IPC_TIMEOUT));
	printk("host config: "); cavs_hda_dbg(host_in, STREAM_ID);

	cavs_hda_init(host_in, STREAM_ID);
	printk("dsp init: "); cavs_hda_dbg(host_in, STREAM_ID);

	int res = cavs_hda_set_buffer(host_in, STREAM_ID, hda_fifo, FIFO_SIZE);
	printk("dsp set_buffer: "); cavs_hda_dbg(host_in, STREAM_ID);
	zassert_ok(res, "Expected set buffer to succeed");

	cavs_hda_enable(host_in, STREAM_ID);
	printk("dsp enable: "); cavs_hda_dbg(host_in, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_START, STREAM_ID, IPC_TIMEOUT));
	printk("host start: "); cavs_hda_dbg(host_in, STREAM_ID);

	cavs_hda_inc_pos(host_in, STREAM_ID, FIFO_SIZE);
	printk("dsp inc_pos: "); cavs_hda_dbg(host_in, STREAM_ID);

	WAIT_FOR(cavs_hda_wp_rp_eq(host_in, STREAM_ID));
	printk("dsp wp_rp_eq: "); cavs_hda_dbg(host_in, STREAM_ID);

	k_sleep(K_MSEC(10));

	uint32_t msg_count = msg_cnt;
	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_VALIDATE, STREAM_ID, IPC_TIMEOUT));

	WAIT_FOR(msg_cnt > msg_count);
	zassert_true(msg_res == 1, "Expected data validation to be true from Host");

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_RESET, STREAM_ID, IPC_TIMEOUT));
	cavs_hda_disable(host_in, STREAM_ID);
}

void test_hda_host_out_smoke(void)
{
	printk("smoke testing hda with fifo buffer at address %p, size %d\n", hda_fifo, FIFO_SIZE);

	cavs_ipc_set_message_handler(CAVS_HOST_DEV, ipc_message, NULL);
	cavs_ipc_set_done_handler(CAVS_HOST_DEV, ipc_done, NULL);

	struct cavs_hda_streams *host_out = &cavs_hda.host_out;

	printk("Using buffer of size %d at addr %p\n", FIFO_SIZE, hda_fifo);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_RESET, (STREAM_ID + 7), IPC_TIMEOUT));
	printk("host reset: "); cavs_hda_dbg(host_out, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_CONFIG, (STREAM_ID + 7) | (FIFO_SIZE << 8), IPC_TIMEOUT));
	printk("host config: "); cavs_hda_dbg(host_out, STREAM_ID);

	k_sleep(K_MSEC(10));

	cavs_hda_init(host_out, STREAM_ID);
	printk("dsp init: "); cavs_hda_dbg(host_out, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_SEND, (STREAM_ID + 7) | (FIFO_SIZE << 8), IPC_TIMEOUT));
	printk("host send: "); cavs_hda_dbg(host_out, STREAM_ID);

	int res = cavs_hda_set_buffer(host_out, STREAM_ID, hda_fifo, FIFO_SIZE);
	printk("dsp set buffer: "); cavs_hda_dbg(host_out, STREAM_ID);
	zassert_ok(res, "Expected set buffer to succeed");


	cavs_hda_enable(host_out, STREAM_ID);
	printk("dsp enable: "); cavs_hda_dbg(host_out, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_START, (STREAM_ID + 7), IPC_TIMEOUT));
	printk("host start: "); cavs_hda_dbg(host_out, STREAM_ID);

	WAIT_FOR(cavs_hda_buf_full(host_out, STREAM_ID));
	printk("dsp wait for full: "); cavs_hda_dbg(host_out, STREAM_ID);

	/* The buffer is in the cached address range and must be invalidated prior to reading. */
	z_xtensa_cache_inv(hda_fifo, FIFO_SIZE);
	bool is_ramp = true;
	for (int j = 0; j < FIFO_SIZE; j++) {
		printk("hda_fifo[%d] = %d\n", j, hda_fifo[j]);
		if (hda_fifo[j] != j) {
			is_ramp = false;
		}
	}
	zassert_true(is_ramp, "Expected data to be a ramp");

	cavs_hda_inc_pos(host_out, STREAM_ID, FIFO_SIZE);
	printk("dsp inc pos: "); cavs_hda_dbg(host_out, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_RESET, (STREAM_ID + 7), IPC_TIMEOUT));
	printk("host reset: "); cavs_hda_dbg(host_out, STREAM_ID);

	cavs_hda_disable(host_out, STREAM_ID);
	printk("dsp disable: "); cavs_hda_dbg(host_out, STREAM_ID);
}
