/*
 * Copyright (c) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/mctp/mctp_i3c.h>
#include <crc-16-ccitt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mctp_i3c, CONFIG_MCTP_LOG_LEVEL);

#define MCTP_I3C_REVISION     0x01


static const MCTP_I3C_STATE_STRING[] = {
	""
}
/**
 * I3C controller binding works by sending data normally and expecting IBI with data
 * from targets sending data.
 */

static inline struct mctp_binding_i3c *binding_to_i3c(struct mctp_binding *b)
{
	return (struct mctp_binding_i3c *)b;
}

static void mctp_i3c_finish_pkt(struct mctp_binding_i3c *i3c, bool valid)
{
	struct mctp_pktbuf *pkt = i3c->rx_pkt;


	if (valid) {
		__ASSERT_NO_MSG(pkt);

		mctp_bus_rx(&i3c->binding, pkt);
	}

	i3c->rx_pkt = NULL;
}

static void mctp_i3c_start_pkt(struct mctp_binding_i3c *i3c, uint8_t len)
{
	__ASSERT_NO_MSG(i3c->rx_pkt == NULL);

	i3c->rx_pkt = mctp_pktbuf_alloc(&i3c->binding, len);

	__ASSERT_NO_MSG(i3c->rx_pkt);
}

/*
 * Each byte coming from the i3c is run through this state machine which
 * does the MCTP packet decoding.
 *
 * The actual packet and buffer being read into is owned by the binding!
 */
static void mctp_i3c_consume(struct mctp_binding_i3c *i3c, uint8_t c)
{
	struct mctp_pktbuf *pkt = i3c->rx_pkt;
	bool valid = false;

	LOG_DBG("i3c consume start state: %d:%s, char 0x%02x", i3c->rx_state,
		MCTP_I3C_STATE_STRING[i3c->rx_state], c);

	__ASSERT_NO_MSG(!pkt == (i3c->rx_state == STATE_WAIT_IBI ||
				 i3c->rx_state == STATE_WAIT_REVISION));

	switch (i3c->rx_state) {
	case STATE_WAIT_REVISION:
		if (c == MCTP_I3C_REVISION) {
			i3c->rx_state = STATE_WAIT_SOURCE;
			i3c->rx_fcs_calc = crc_16_ccitt_byte(FCS_INIT_16, c);
		} else if (c == MCTP_I3C_FRAMING_FLAG) {
			/* Handle the case where there are bytes dropped in request,
			 * and the state machine is out of sync. The failed request's
			 * trailing footer i.e. 0x7e would be interpreted as next
			 * request's framing footer. So if we are in STATE_WAIT_REVISION
			 * and receive 0x7e byte, then continue to stay in
			 * STATE_WAIT_REVISION
			 */
			LOG_DBG("Received serial framing flag 0x%02x while waiting"
				" for serial revision 0x%02x.",
				c, MCTP_I3C_REVISION);
		} else {
			LOG_DBG("invalid revision 0x%02x", c);
			i3c->rx_state = STATE_WAIT_SYNC_START;
		}
		break;
	case STATE_WAIT_LEN:
		if (c > i3c->binding.pkt_size || c < sizeof(struct mctp_hdr)) {
			LOG_DBG("invalid size %d", c);
			i3c->rx_state = STATE_WAIT_SYNC_START;
		} else {
			mctp_i3c_start_pkt(i3c, 0);
			pkt = i3c->rx_pkt;
			i3c->rx_exp_len = c;
			i3c->rx_state = STATE_DATA;
			i3c->rx_fcs_calc = crc_16_ccitt_byte(i3c->rx_fcs_calc, c);
		}
		break;
	case STATE_DATA:
		if (c == MCTP_I3C_ESCAPE) {
			i3c->rx_state = STATE_DATA_ESCAPED;
		} else {
			mctp_pktbuf_push(pkt, &c, 1);
			i3c->rx_fcs_calc = crc_16_ccitt_byte(i3c->rx_fcs_calc, c);
			if (pkt->end - pkt->mctp_hdr_off == i3c->rx_exp_len) {
				i3c->rx_state = STATE_WAIT_FCS1;
			}
		}
		break;
	case STATE_DATA_ESCAPED:
		c ^= 0x20;
		mctp_pktbuf_push(pkt, &c, 1);
		i3c->rx_fcs_calc = crc_16_ccitt_byte(i3c->rx_fcs_calc, c);
		if (pkt->end - pkt->mctp_hdr_off == i3c->rx_exp_len) {
			i3c->rx_state = STATE_WAIT_FCS1;
		} else {
			i3c->rx_state = STATE_DATA;
		}
		break;

	case STATE_WAIT_FCS1:
		i3c->rx_fcs = c << 8;
		i3c->rx_state = STATE_WAIT_FCS2;
		break;
	case STATE_WAIT_FCS2:
		i3c->rx_fcs |= c;
		i3c->rx_state = STATE_WAIT_SYNC_END;
		break;
	case STATE_WAIT_SYNC_END:
		if (i3c->rx_fcs == i3c->rx_fcs_calc) {
			if (c == MCTP_I3C_FRAMING_FLAG) {
				valid = true;
			} else {
				valid = false;
				LOG_DBG("missing end frame marker");
			}
		} else {
			valid = false;
			LOG_DBG("invalid fcs : 0x%04x, expect 0x%04x", i3c->rx_fcs,
				i3c->rx_fcs_calc);
		}

		mctp_i3c_finish_pkt(i3c, valid);
		i3c->rx_state = STATE_WAIT_SYNC_START;
		break;
	}

	LOG_DBG("i3c consume end state: %d:%s, char 0x%02x", i3c->rx_state,
		MCTP_STATE_STRING[i3c->rx_state], c);
}

static void mctp_i3c_callback(const struct device *dev, struct i3c_event *evt, void *userdata)
{
	struct mctp_binding_i3c *binding = userdata;

	switch (evt->type) {
	case I3C_TX_DONE:
		binding->tx_res = 0;
		break;
	case I3C_TX_ABORTED:
		binding->tx_res = -EIO;
		break;
	case I3C_RX_RDY:
		/* buffer being read into is ready */
		binding->rx_res = evt->data.rx.len;
		/* parse the buffer */
		for (size_t i = 0; i < evt->data.rx.len; i++) {
			mctp_i3c_consume(binding, evt->data.rx.buf[evt->data.rx.offset + i]);
		}
		break;
	case I3C_RX_BUF_REQUEST:
		for (int i = 0; i < sizeof(binding->rx_buf_used); i++) {
			if (!binding->rx_buf_used[i]) {
				binding->rx_buf_used[i] = true;
				i3c_rx_buf_rsp(dev, binding->rx_buf[i],
						sizeof(binding->rx_buf[i]));
				break;
			}
		}
		break;
	case I3C_RX_BUF_RELEASED:
		for (int i = 0; i < sizeof(binding->rx_buf_used); i++) {
			if (binding->rx_buf[i] == evt->data.rx_buf.buf) {
				binding->rx_buf_used[i] = false;
				break;
			}
		}
		break;
	case I3C_RX_STOPPED:
		break;
	case I3C_RX_DISABLED:
		break;
	}
}

void mctp_i3c_start_rx(struct mctp_binding_i3c *i3c)
{

}

/*
 * MCTP over I3C is very simple, bytes contain (in order here)
 * 0 = i3c device addr
 * 1 = mctp version header
 * 2 = mctp destination id
 * 3 = mctp source id
 * 4 = packet info (som/eom/pkt_seq/to/msg_tag)
 * 5 = Integrity Check Bit + Message Type
 * 6:N-1 = N byte message data with optional integrity checksum
 * N Packet Error Code [PEC]
 */
int mctp_i3c_tx(struct mctp_binding *b, struct mctp_pktbuf *pkt)
{
	struct mctp_binding_i3c *i3c = binding_to_i3c(b);
	uint8_t *buf;
	size_t len;

}

int mctp_i3c_start(struct mctp_binding *binding)
{
	/* Set the i3c target IBI callback */
	/* TODO i3c_dev->ibi_cb = ...; */

	/* Enable IBI */
	/* TODO i3c_enable_ibi(...); */

	/* TODO Now when IBI occurs an asynchroous read will need to be started */

	return 0;
}
