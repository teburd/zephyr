/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <ztest.h>
#include <cavs_ipc.h>
#include <cavs_hda.h>
#include "tests.h"

#define FIFO_SIZE 128
#define TRANSFER_COUNT 1024

__attribute__((section(".dma_buffers"))) uint8_t in_fifo[128];

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


void test_hda_host_in(void)
{

	/* reset this just in case... */
	cavs_ipc_init(CAVS_HOST_DEV);
	cavs_ipc_set_message_handler(CAVS_HOST_DEV, ipc_message, NULL);
	cavs_ipc_set_done_handler(CAVS_HOST_DEV, ipc_done, NULL);

	/**
	 * Tell the host to setup an hda host in stream with the given id (0) and size of 1024
	 * We can add more flags/options here if needed
	 */
	while(!cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_INIT, 0 | (1024 << 8), K_FOREVER)) {}

	struct cavs_hda_streams *host_in = &cavs_hda.host_in;

	/*
	 * Write out a message byte by byte to the stream
	 */
	cavs_hda_set_buffer(host_in, 0, in_fifo, FIFO_SIZE, 1);
	cavs_hda_enable(host_in, 0);

	/**
	 * Tell the host to set run bit for hda host in stream with the given id (0)
	 */

	/* Must tell the DMA to exist L1 before setting RUN bit otherwise its gets
	 * stuck, maybe the host side can do this?
	 */
	/*  cavs_hda_l1_exit(host_in, 0); */

	printk("sending host in run and waiting for done flag\n");
	while(!cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_RUN, 0, K_FOREVER)) {};

	printk("writing...\n");


	int res;
	for(uint8_t i = 0; i < 1024; i++) {
		res = cavs_hda_write(host_in, 0, i);
		zassert_true(res > 0, "cavs hda write failed");
	}

	/*
	 * Tell the host to validate the stream containing a counter up to the given value
	 * and respond yes/no
	 */
	while(!cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_VALIDATE, 127, K_FOREVER)) {}
	/*zassert_eq(msg_res == 1, "Validation of data sent from dsp to host over HDA stream failed"); */

	while(!cavs_ipc_send_message_sync(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_RESET, 127, K_FOREVER)) {}

	/* disable the stream */
	cavs_hda_disable(host_in, 0);
}
