/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <ztest.h>
#include <cavs_ipc.h>
#include <cavs_hda.h>
#include "tests.h"

#define FIFO_SIZE 64
#define TRANSFER_COUNT (32*5)

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


#define STREAM_ID 3

void test_hda_host_in(void)
{

	printk("hda host in test, setting handlers\n");
	/* reset this just in case... */
	cavs_ipc_set_message_handler(CAVS_HOST_DEV, ipc_message, NULL);
	cavs_ipc_set_done_handler(CAVS_HOST_DEV, ipc_done, NULL);

	printk("sending host init\n");

	/**
	 * Tell the host to setup an hda host in stream with the given id (0) and size of FIFO_SIZE
	 * We can add more flags/options here if needed
	 */
	WAIT_FOR(cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_INIT, STREAM_ID| (FIFO_SIZE << 8)));
	k_msleep(1);

	struct cavs_hda_streams *host_in = &cavs_hda.host_in;

	printk("set buffer and enable\n");

	/*
	 * Write out a message byte by byte to the stream
	 */
	cavs_hda_set_buffer(host_in, STREAM_ID, in_fifo, FIFO_SIZE, 1);
	cavs_hda_enable(host_in, STREAM_ID);

	/**
	 * Tell the host to set run bit for hda host in stream with the given id (0)
	 */

	/* Must tell the DMA to exist L1 before setting RUN bit otherwise its gets
	 * stuck, maybe the host side can do this?
	 */
	/*  cavs_hda_l1_exit(host_in, 0); */

	printk("sending hda host start\n");
	WAIT_FOR(cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_RUN, STREAM_ID));
	k_msleep(1);
	printk("hda host start sent");

	printk("writing...\n");

	int res;
	for(uint32_t i = 0; i < TRANSFER_COUNT; i++) {
		printk("writing %d\n", i);
		res = cavs_hda_write(host_in, STREAM_ID, i & 0xff);
		zassert_true(res > 0, "cavs hda write failed");
		printk("forcing l1 exit\n");
		cavs_hda_l1_exit(host_in, STREAM_ID);
	}

	printk("write done\n");

	/*
	 * Tell the host to validate the stream containing a counter up to the given value
	 * and respond yes/no
	 */
	WAIT_FOR(cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_VALIDATE, 127));
	WAIT_FOR(cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_RESET, 127));

	/* disable the stream */
	cavs_hda_disable(host_in, 0);
}
