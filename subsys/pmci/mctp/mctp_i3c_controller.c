/*
 * Copyright (c) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/sys/util.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pmci/mctp/mctp_i3c_controller.h>
#include <zephyr/pmci/mctp/mctp_i3c_endpoint.h>
#include <crc-16-ccitt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mctp_i3c_controller, CONFIG_MCTP_LOG_LEVEL);



static inline void mctp_i3c_recv_msg(struct mctp_binding_i3c_controller *binding, size_t endpoint_idx)
{
	uint8_t rx_buf[256];

	/* Callback already done *in a work queue* dedicated to i3c but is shared
	 * among all i3c buses. Likely only one per device anyways since its a
	 * beastly IP block so no need to requeue the request
	 */
	struct i3c_msg msg = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
		.flags = I3C_MSG_READ | I3C_MSG_STOP,
	};

	int rc = i3c_transfer(binding->endpoint_i3c_devs[endpoint_idx], &msg, 1);

	if (rc != 0) {
		LOG_ERR("Error requesting read from endpoint %d", endpoint_idx);
		return;
	}

	struct mctp_pktbuf *pkt = mctp_pktbuf_alloc(&binding->binding, msg.num_xfer);

	if (pkt == NULL) {
		LOG_ERR("Out of memory trying to allocate mctp pktbuf when receiving message from endpoint %d", endpoint_idx);
		return;
	}

	memcpy(pkt->data, msg.buf, msg.num_xfer);

	/* pkt is moved to mctp and no longer owned by the binding */
	mctp_bus_rx(&binding->binding, pkt);
}

/* DMTF mandatory byte with IBI signaling a pending Read */
#define MCTP_I3C_MDB_PENDING_READ 0xAE

void mctp_i3c_ibi_cb(struct i3c_device_desc *target,
                     struct i3c_ibi_payload *payload)
{
	struct mctp_binding_i3c_controller *binding = mctp_i3c_endpoint_binding(target->dev);
	int endpoint_idx = -1;

	for (int i = 0; i < binding->num_endpoints; i++) {
		if (binding->devices[i] == target->dev) {
			endpoint_idx = i;
			break;
		}
	}

	if (endpoint_idx == -1) {
		LOG_ERR("IBI from unknown I3C Device, maybe missing in devicetree? %p", target->dev);
		return;
	}

	uint8_t endpoint_id = binding->endpoint_ids[endpoint_idx];

	/* Endpoints have states as i3c has a whole handshake flow it expects, an IBI
	 * could mean any number of things depending on the state of the endpoint
	 */
	switch (binding->endpoint_states[endpoint_idx]) {
		case MCTP_I3C_AWAIT_VERSION_RESP:
			/* Expect a version response command here */
			LOG_DBG("Got version response from %d\n", endpoint_idx);
			break;
		case MCTP_I3C_AWAIT_SET_ENDPOINT_RESP:
			/* Expect a response to setting the endpoint id */
			LOG_DBG("Got set endpoint id response from %d\n", endpoint_idx);
			break;
		case MCTP_I3C_AWAIT_MSG:
			/* I3C should have an IBI MDB of 0xAE according to DMTF */
			if (payload->payload_len >= 1 && payload->payload[0] == MCTP_I3C_MDB_PENDING_READ)
			{
				mctp_i3c_recv_msg(binding, endpoint_idx);
			} else {
				LOG_WRN("Expected a IBI payload with the mandatory pending read byte, something broke");
			}
			break;
		default:
			LOG_WRN("Unknown state of endpoint %d, IBI received\n", endpoint_idx);
			break;
	}
}

int mctp_i3c_controller_tx(struct mctp_binding *binding, struct mctp_pktbuf *pkt)
{
	/* Which i2c device am I sending this to? */
	struct mctp_hdr *hdr = mctp_pktbuf_hdr(pkt);
	struct mctp_binding_i3c_controller *b =
		CONTAINER_OF(binding, struct mctp_binding_i3c_controller, binding);
	uint8_t pktsize = pkt->end - pkt->start;
	int endpoint_idx = -1;

	for (int i = 0; i < b->num_endpoints; i++) {
		if (b->endpoint_ids[i] == hdr->dest) {
			endpoint_idx = i;
			break;
		}
	}

	if (endpoint_idx == -1) {
		LOG_ERR("Invalid endpoint id %d when sending message", hdr->dest);
		return 0;
	}

	if (b->endpoint_states[endpoint_idx] != MCTP_I3C_AWAIT_MSG) {
		LOG_ERR("Attempted to send message to endpoint before it was ready!");
		return 0;
	}

	struct i3c_msg msg = {
		.buf = &pkt->data[pkt->start],
		.len = pktsize,
	};

	LOG_DBG("sending message");
	int rc = i3c_transfer(b->endpoint_i3c_devs[endpoint_idx], &msg, 1);

	if (rc != 0) {
		LOG_WRN("Failed sending message to endpoint %d, result %d", hdr->dest, rc);
	}

	/* We must *always* return 0 despite errors, otherwise libmctp does not free the packet! */
	return 0;
}

#define MCTP_HDR_VER 0x01
#define MCTP_NULL_ADDR 0x00
#define MCTP_CONTROL_ID 0x00
#define MCTP_GET_VERSION_CODE 0xFE
const uint8_t MCTP_GET_VER_CMD[] = { MCTP_HDR_VER, MCTP_NULL_ADDR, MCTP_NULL_ADDR, 0x00, MCTP_CONTROL_ID, MCTP_GET_VERSION_CODE, 0xFF };

int mctp_i3c_controller_start(struct mctp_binding *binding)
{
	int rc = 0;

	struct mctp_binding_i3c_controller *b =
		CONTAINER_OF(binding, struct mctp_binding_i3c_controller, binding);

	if (rc != 0) {
		LOG_WRN("could not do dynamic address assignment");

	}

	/* Initial flow of mctp over i3c wants us to request the MCTP version
	 * and then get back the version from an IBI with a payload.
	 */
	struct i3c_msg msg = {
		.buf = (uint8_t *)MCTP_GET_VER_CMD,
		.len = sizeof(MCTP_GET_VER_CMD),
	};

	/* Get the MCTP version of the i3c target by requesting it using physical bus addressing only and expecting the base
	 * version information returned (0xFF)
	 */
	for (int i = 0; i < b->num_endpoints; i++) {
		mctp_i3c_endpoint_bind(b->devices[i], b, &b->endpoint_i3c_devs[i]);
		rc = i3c_ibi_enable(b->endpoint_i3c_devs[i]);
		if (rc != 0) {
			LOG_WRN("Could not enable IBI for I3C PID %llx",
			        (uint64_t)b->endpoint_i3c_devs[i]->pid);
			continue;
		}
		rc = i3c_transfer(b->endpoint_i3c_devs[i], &msg, 1);
		if (rc != 0) {
			LOG_WRN("Could not transfer request of MCTP version to I3C PID %llx",
			        (uint64_t)b->endpoint_i3c_devs[i]->pid);
		}
	}

	mctp_binding_set_tx_enabled(binding, true);

	LOG_DBG("started");

	return 0;
}
