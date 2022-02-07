/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <ztest.h>
#include <cavs_ipc.h>
#include <cavs_hda.h>
#include "tests.h"

#define IPC_TIMEOUT K_MSEC(100)
#define STREAM_ID 1
#define FIFO_SIZE 256
#define TRANSFER_SIZE 256
#define TRANSFER_COUNT 100

__attribute__((section(".dma_buffers"))) uint8_t in_fifo[FIFO_SIZE];

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

	printk("smoke testing hda with fifo buffer at address %p, size %d\n", in_fifo, FIFO_SIZE);

	cavs_ipc_set_message_handler(CAVS_HOST_DEV, ipc_message, NULL);
	cavs_ipc_set_done_handler(CAVS_HOST_DEV, ipc_done, NULL);

	/**
	 * Tell the host to setup an hda host in stream with the given id (0) and size of FIFO_SIZE
	 * We can add more flags/options here if needed
	 */

	struct cavs_hda_streams *host_in = &cavs_hda.host_in;

	printk("Using buffer of size %d at addr %p\n", FIFO_SIZE, in_fifo);

	for(uint32_t i = 0; i < FIFO_SIZE; i++) {
		in_fifo[i] = i & 0xff;
	}

	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_INIT, STREAM_ID | (FIFO_SIZE << 8), IPC_TIMEOUT));
	cavs_hda_set_buffer(host_in, STREAM_ID, in_fifo, FIFO_SIZE);

	/* Enable the gateway */
	cavs_hda_enable(host_in, STREAM_ID);

	/**
	 * Tell the host to set run bit for hda host in stream with the given id (0)
	 */
	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_START, STREAM_ID, IPC_TIMEOUT));

	cavs_hda_post_write(host_in, STREAM_ID, FIFO_SIZE);

	/*
	 * Tell the host to validate the stream containing a counter up to the given value
	 * and respond yes/no
	 */
	WAIT_FOR(cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_VALIDATE, STREAM_ID));
	WAIT_FOR(cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_RESET, STREAM_ID, IPC_TIMEOUT));

	/* disable the stream */
	cavs_hda_disable(host_in, STREAM_ID);
}
