/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <kernel.h>
#include <ztest.h>
#include <cavs_ipc.h>
#include <cavs_hda.h>

#include "tests.h"

__attribute__((section("dma_buffers"))) uint8_t host_in_buf[32];

void test_hda_host_in(void)
{
	bool ipc_ret;

	/**
	 * Tell the host to setup a hda host in stream with the given id (0)
	 */
	ipc_ret = cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_SETUP, 0);
	zassert_true(ipc_ret, "send failed");

	struct cavs_hda_streams *host_in = &cavs_hda.host_in;

	/*
	 * Write out a message byte by byte to the stream
	 */
	cavs_hda_set_buffer(host_in, 0, host_in_buf, 32, 1);

	/**
	 * Tell the host to start hda host in stream with the given id (0)
	 */
	ipc_ret = cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_START, 0);
	zassert_true(ipc_ret, "send failed");


	cavs_hda_enable(host_in, 0);

	for(uint8_t i = 0; i < 128; i++) {
		cavs_hda_write(hda, sid, i);
	}

	/*
	 * Tell the host to validate the stream containing a counter up to the given value
	 * and respond yes/no
	 */
	ipc_ret = cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_VALIDATE, 127);
	zassert_true(ipc_ret, "send failed");

	ipc_ret = cavs_ipc_send_message(CAVS_HOST_DEV, IPCCMD_HDA_HOST_IN_RESET, 127);
	zassert_true(ipc_ret, "send failed");


	/* disable the stream */
	cavs_hda_disable(host_in, 0);

}
