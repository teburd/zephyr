/*
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

/** Required to access Timeout Queue APIs, which are used instead of the
 * Timer APIs because of concerns on size on rtio_sqe (k_timer is more
 * than double the size of _timeout). Users will have to instantiate a
 * pool of SQE objects, thus its size directly impacts memory footprint
 * of RTIO applications.
 */
#include <../kernel/include/timeout_q.h>

#include "rtio_sched.h"

#ifdef CONFIG_RTIO_OP_DELAY
static void rtio_sched_alarm_expired(struct _timeout *t)
{
	struct rtio_sqe *sqe = CONTAINER_OF(t, struct rtio_sqe, delay.to);

	rtio_iodev_sqe_ok(sqe, 0);
}

void rtio_sched_alarm(struct rtio_sqe *sqe, k_timeout_t timeout)
{
	z_init_timeout(&sqe->delay.to);
	z_add_timeout(&sqe->delay.to, rtio_sched_alarm_expired, timeout);
}
#endif /* CONFIG_RTIO_OP_DELAY */
