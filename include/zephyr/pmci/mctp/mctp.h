/*
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_MCTP_H_
#define ZEPHYR_MCTP_H_

#include <stdint.h>
#include <libmctp.h>

/**
 * @brief Register a bus with the global MCTP context
 */
int zephyr_mctp_register_bus(struct mctp_binding *binding);

/**
 * @brief Open a socket-like connection to a MCTP peer
 */
int zephyr_mctp_open(uint8_t endpoint_id);

/**
 * @brief Close a MCTP socket
 */
int zephyr_mctp_close(int sock);

/**
 * @brief Get the MCTP peer endpoint for a handle
 */
int zephyr_mctp_endpoint(int sock, uint8_t *endpoint_id);

/**
 * @brief Write to a socket like handle for a connection to a peer over MCTP
 */
int zephyr_mctp_write(int sock, uint8_t *msg, size_t len);

/**
 * @brief Read from a socket like handle up to some number of available bytes
 */
int zephyr_mctp_read(int sock, uint8_t *msg, size_t *len);

/**
 * @brief Read exactly the number of bytes for an endpoint
 */
int zephyr_mctp_read_exact(int sock, uint8_t *msg, size_t len);

#endif /* ZEPHYR_MCTP_H */
