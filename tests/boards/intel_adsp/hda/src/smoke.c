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

static volatile int msg_res;


static bool ipc_message(const struct device *dev, void *arg,
			uint32_t data, uint32_t ext_data)
{
	msg_res = data;
	return true;
}

static void ipc_done(const struct device *dev, void *arg)
{
	printk("ipc done called\n");
}



void test_hda_smoke(void)
{

	printk("smoke testing hda with fifo buffer at address %p, size %d\n", hda_fifo, FIFO_SIZE);

	cavs_ipc_set_message_handler(CAVS_HOST_DEV, ipc_message, NULL);
	cavs_ipc_set_done_handler(CAVS_HOST_DEV, ipc_done, NULL);

	struct cavs_hda_streams *host_in = &cavs_hda.host_in;

	printk("Using buffer of size %d at addr %p\n", FIFO_SIZE, hda_fifo);

	for(uint32_t i = 0; i < FIFO_SIZE; i++) {
		hda_fifo[i] = i & 0xff;
	}
	/* The buffer is in the cached address range and must be flushed when done writing. */
	z_xtensa_cache_flush(hda_fifo, FIFO_SIZE);

	cavs_hda_set_buffer(host_in, STREAM_ID, hda_fifo, FIFO_SIZE);
	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_INIT, STREAM_ID | (FIFO_SIZE << 8), IPC_TIMEOUT));
	cavs_hda_enable(host_in, STREAM_ID);
	cavs_hda_dbg(host_in, STREAM_ID);
	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_START, STREAM_ID, IPC_TIMEOUT));
	cavs_hda_post_write(host_in, STREAM_ID, FIFO_SIZE);
	cavs_hda_dbg(host_in, STREAM_ID);

	WAIT_FOR(cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_VALIDATE, STREAM_ID));
	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_RESET, STREAM_ID, IPC_TIMEOUT));
	cavs_hda_disable(host_in, STREAM_ID);
}
