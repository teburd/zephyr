/*
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MCTP_I3C_H_
#define ZEPHYR_MCTP_I3C_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <libmctp.h>

/**
 * @brief An MCTP binding for Zephyr's I3C interface
 */
struct mctp_binding_i3c {
	/** @cond INTERNAL_HIDDEN */
	struct mctp_binding binding;

	/* i3c device we are talking to */
	const struct i3c_device_desc *i3c_device;

	/* Is this binding acting as a controller or target? */
	bool i3c_target_binding;

	/* receive buffer and state */
	uint8_t rx_buf[256];
	struct mctp_pktbuf *rx_pkt;
	enum {
		STATE_WAIT_IBI,
		STATE_WAIT_REVISION,
		STATE_WAIT_SRC_EID,
		STATE_WAIT_DEST_EID,
		STATE_WAIT_PKT_SEQ,
		STATE_WAIT_MSG_HDR,
		STATE_DATA,
		STATE_WAIT_PEC,
	} rx_state;
	int rx_res;

	/* staging buffer for tx */
	uint8_t tx_buf[256];
	int tx_res;

	/** @endcond INTERNAL_HIDDEN */
};

/**
 * @brief Start listening for MCTP messages over I3C
 *
 * @param i3c MCTP I3C binding
 */
void mctp_i3c_start_rx(struct mctp_binding_i3c *i3c);

/** @cond INTERNAL_HIDDEN */
int mctp_i3c_start(struct mctp_binding *binding);
int mctp_i3c_tx(struct mctp_binding *binding, struct mctp_pktbuf *pkt);
/** @endcond INTERNAL_HIDDEN */

/**
 * @brief Statically define a MCTP bus binding for a I3C
 *
 * @param _name Symbolic name of the bus binding variable
 * @param _i3c_device I3C device descriptor to bind
 * @param _target_mode If this binding is acting in target mode set to true
 */
#define MCTP_I3C_DEFINE(_name, _i3c_device, _target_mode)                                          \
	struct mctp_binding_uart _name = {                                                         \
		.binding =                                                                         \
			{                                                                          \
				.name = STRINGIFY(_name), .version = 1,                            \
						  .pkt_size = MCTP_PACKET_SIZE(MCTP_BTU),          \
						  .pkt_header = 0, .pkt_trailer = 0,               \
						  .start = mctp_uart_start, .tx = mctp_uart_tx,    \
				},                                                                 \
				.i3c_device = _i3c_device,                                         \
				.i3c_target_binding = _target_mode,                                \
				.rx_state = _target_mode ? STATE_WAIT_IBI : STATE_WAIT_REVISION,   \
				.rx_pkt = NULL,                                                    \
				.rx_res = 0,                                                       \
				.tx_res = 0,                                                       \
	};

#endif /* ZEPHYR_MCTP_I3C_H_ */
