/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Infineon Technologies AG,
 * SPDX-FileCopyrightText: or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Interfaces internal to the RTIO subsystem
 */

#ifndef ZEPHYR_SUBSYS_RTIO_RTIO_INTERNAL_H_
#define ZEPHYR_SUBSYS_RTIO_RTIO_INTERNAL_H_

#include <zephyr/rtio/rtio.h>

#ifdef CONFIG_RTIO_TIMEOUT_IODEV
/**
 * @brief Take a submission out of the timeout iodev without completing it
 *
 * Used by the executor to disarm a deadline whose chain has ended. The
 * submission is unlinked from the timeout queue if it is still pending there,
 * which is also what stops the expiry handler from taking it afterwards, and the
 * shared kernel timeout is re-armed to whatever remains.
 *
 * @param iodev_sqe Submission dispatched to the timeout iodev
 */
void rtio_timeout_disarm(struct rtio_iodev_sqe *iodev_sqe);
#else
static inline void rtio_timeout_disarm(struct rtio_iodev_sqe *iodev_sqe)
{
	ARG_UNUSED(iodev_sqe);
}
#endif

/**
 * @brief Record the result a deadline reports once the chain it guards has ended
 *
 * The verdict is kept in the deadline's own runtime state, which only exists
 * when the operation is enabled; with it disabled nothing ever reaches a guard
 * path and these are both dead code.
 *
 * @param guard  Deadline submission
 * @param result Result to report
 */
static inline void rtio_guard_set_verdict(struct rtio_iodev_sqe *guard, int result)
{
#ifdef CONFIG_RTIO_OP_DEADLINE
	guard->rt.timeout.result = result;
#else
	ARG_UNUSED(guard);
	ARG_UNUSED(result);
#endif
}

/**
 * @brief The result a deadline is to report
 *
 * @param guard Deadline submission
 * @return The recorded result, 0 if none was recorded
 */
static inline int rtio_guard_verdict(const struct rtio_iodev_sqe *guard)
{
#ifdef CONFIG_RTIO_OP_DEADLINE
	return guard->rt.timeout.result;
#else
	ARG_UNUSED(guard);
	return 0;
#endif
}

#endif /* ZEPHYR_SUBSYS_RTIO_RTIO_INTERNAL_H_ */
