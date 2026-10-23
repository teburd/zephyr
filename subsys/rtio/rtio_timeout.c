/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Infineon Technologies AG,
 * SPDX-FileCopyrightText: or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RTIO timeout iodev backing the RTIO_OP_DELAY and RTIO_OP_DEADLINE operations.
 *
 * Both are handled like any other iodev operation rather than as an executor
 * special case. The "device" here is the system clock: pending submissions are
 * kept in a per-instance list sorted by absolute expiration and driven by a
 * single kernel timeout re-armed to the nearest expiration. The
 * absolute expiration lives in the iodev_sqe wrapper's runtime scratch (a
 * single k_timepoint_t, no embedded struct _timeout), keeping the submission
 * description immutable, and reduces the kernel timeout queue to one node per
 * timeout iodev instance.
 */

#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

#include "rtio_internal.h"

/** Required to access the Timeout Queue APIs, which are used instead of the
 * Timer APIs because of concerns on size: a shared struct _timeout is cheaper
 * than a k_timer, and RTIO applications are sensitive to memory footprint.
 */
#include <../kernel/include/timeout_q.h>

/**
 * @brief Per-instance state for a timeout iodev
 *
 * The sorted list link reuses each iodev_sqe's embedded mpsc_node (@ref
 * rtio_iodev_sqe.q), which is free while the submission is owned by this iodev
 * (it is out of the context submission queue and not yet recycled to the pool).
 */
struct rtio_timeout_iodev_data {
	struct k_spinlock lock;
	struct _timeout timeout;     /* single kernel timeout shared by all pending delays */
	struct rtio_iodev_sqe *head; /* soonest-first sorted list of pending delays */
};

/* Reuse the iodev_sqe's mpsc_node storage as the sorted-list link. */
static inline struct rtio_iodev_sqe *rtio_tq_next(struct rtio_iodev_sqe *iodev_sqe)
{
	struct mpsc_node *node = mpsc_ptr_get(iodev_sqe->q.next);

	return node == NULL ? NULL : CONTAINER_OF(node, struct rtio_iodev_sqe, q);
}

static inline void rtio_tq_set_next(struct rtio_iodev_sqe *iodev_sqe, struct rtio_iodev_sqe *next)
{
	mpsc_ptr_set(iodev_sqe->q.next, next == NULL ? NULL : &next->q);
}

/**
 * @brief Claim a deadline that is being taken out of the list for processing
 *
 * A deadline is finished either by the action that takes it out of this list, or
 * by the chain it guards ending, and the two can run at once on different CPUs.
 * Claiming under @p data->lock, the same lock every removal from the list holds,
 * is what lets the chain see that an action is in flight and leave the teardown
 * to it: a caller that has removed the submission itself knows no action can be
 * started afterwards, and one that finds it already gone sees the claim.
 *
 * Submissions that are not deadlines are finished by their own completion alone
 * and need no claim.
 *
 * @param iodev_sqe Submission being taken out of the list
 */
static inline void rtio_tq_claim(struct rtio_iodev_sqe *iodev_sqe)
{
	if (IS_ENABLED(CONFIG_RTIO_OP_DEADLINE) && iodev_sqe->sqe.op == RTIO_OP_DEADLINE) {
		atomic_or(&iodev_sqe->status, RTIO_SQE_STATUS_FINISHING);
	}
}

/**
 * @brief Insert a submission into the sorted list
 *
 * @retval true if the submission became the new head (nearest deadline)
 */
static bool rtio_tq_insert(struct rtio_timeout_iodev_data *data, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_iodev_sqe *prev = NULL;
	struct rtio_iodev_sqe *curr = data->head;

	while (curr != NULL &&
	       sys_timepoint_cmp(curr->rt.timeout.expiry, iodev_sqe->rt.timeout.expiry) <= 0) {
		prev = curr;
		curr = rtio_tq_next(curr);
	}

	rtio_tq_set_next(iodev_sqe, curr);

	if (prev == NULL) {
		data->head = iodev_sqe;
		return true;
	}

	rtio_tq_set_next(prev, iodev_sqe);
	return false;
}

/**
 * @brief Detach all expired submissions from the head into a NULL-terminated batch
 */
static struct rtio_iodev_sqe *rtio_tq_pop_expired(struct rtio_timeout_iodev_data *data)
{
	struct rtio_iodev_sqe *batch = NULL;
	struct rtio_iodev_sqe *last = NULL;

	while (data->head != NULL && sys_timepoint_expired(data->head->rt.timeout.expiry)) {
		struct rtio_iodev_sqe *iodev_sqe = data->head;

		data->head = rtio_tq_next(iodev_sqe);
		rtio_tq_set_next(iodev_sqe, NULL);
		rtio_tq_claim(iodev_sqe);

		if (last == NULL) {
			batch = iodev_sqe;
		} else {
			rtio_tq_set_next(last, iodev_sqe);
		}
		last = iodev_sqe;
	}

	return batch;
}

/**
 * @brief Remove a specific submission from the sorted list
 *
 * @retval true if @p target was pending and got unlinked
 * @retval false if @p target was not in the list (already expired/completed)
 */
static bool rtio_tq_remove(struct rtio_timeout_iodev_data *data, struct rtio_iodev_sqe *target)
{
	struct rtio_iodev_sqe *prev = NULL;
	struct rtio_iodev_sqe *curr = data->head;

	while (curr != NULL) {
		if (curr == target) {
			struct rtio_iodev_sqe *next = rtio_tq_next(curr);

			if (prev == NULL) {
				data->head = next;
			} else {
				rtio_tq_set_next(prev, next);
			}
			rtio_tq_set_next(curr, NULL);
			return true;
		}
		prev = curr;
		curr = rtio_tq_next(curr);
	}

	return false;
}

static void rtio_tq_expired(struct _timeout *timeout);

/**
 * @brief Arm the shared kernel timeout to the current head, or disarm if empty
 *
 * Must be called with @p data->lock held. On SMP, aborting the timeout may fail
 * with -EAGAIN if the handler is in flight on another CPU; the only correct
 * response is to drop the lock and retry, which is done by the caller.
 *
 * @retval true if the arm completed
 * @retval false if the timeout abort needs the caller to drop the lock and retry
 */
static bool rtio_tq_try_arm(struct rtio_timeout_iodev_data *data)
{
	if (z_try_abort_timeout(&data->timeout) == -EAGAIN) {
		return false;
	}

	if (data->head != NULL) {
		z_add_timeout(&data->timeout, rtio_tq_expired,
			      sys_timepoint_timeout(data->head->rt.timeout.expiry));
	}

	return true;
}

/**
 * @brief Kernel timeout handler for the nearest pending delay
 *
 * Runs in the system clock announce context, exactly as the previous per-SQE
 * alarm did. Expired submissions are detached and the timeout re-armed while
 * holding the lock; completions run with the lock released because completing a
 * submission may re-enter rtio_timeout_iodev_submit() (e.g. a chained delay).
 */
static void rtio_tq_expired(struct _timeout *timeout)
{
	struct rtio_timeout_iodev_data *data =
		CONTAINER_OF(timeout, struct rtio_timeout_iodev_data, timeout);
	struct rtio_iodev_sqe *batch;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	batch = rtio_tq_pop_expired(data);

	if (data->head != NULL) {
		z_add_timeout(&data->timeout, rtio_tq_expired,
			      sys_timepoint_timeout(data->head->rt.timeout.expiry));
	}

	k_spin_unlock(&data->lock, key);

	while (batch != NULL) {
		struct rtio_iodev_sqe *iodev_sqe = batch;

		batch = rtio_tq_next(iodev_sqe);
		rtio_tq_set_next(iodev_sqe, NULL);
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}
}

/**
 * @brief The timeout an operation handled by this iodev asked for
 *
 * The operations this iodev serves each describe their own timeout, in the arg
 * union member their op code selects.
 *
 * @retval K_FOREVER if the op is not one this iodev handles
 */
static k_timeout_t rtio_timeout_of(const struct rtio_iodev_sqe *iodev_sqe)
{
	switch (iodev_sqe->sqe.op) {
#ifdef CONFIG_RTIO_OP_DELAY
	case RTIO_OP_DELAY:
		return iodev_sqe->sqe.delay.timeout;
#endif
#ifdef CONFIG_RTIO_OP_DEADLINE
	case RTIO_OP_DEADLINE:
		return iodev_sqe->sqe.deadline.timeout;
#endif
	default:
		return K_FOREVER;
	}
}

static void rtio_timeout_iodev_submit(struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_timeout_iodev_data *data = iodev_sqe->sqe.iodev->data;
	k_timeout_t timeout = rtio_timeout_of(iodev_sqe);
	k_spinlock_key_t key;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		/* Either an op this iodev does not handle, or one asking to wait
		 * forever, which an expiration cannot express.
		 */
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
		return;
	}

	iodev_sqe->rt.timeout.expiry = sys_timepoint_calc(timeout);

	if (sys_timepoint_expired(iodev_sqe->rt.timeout.expiry)) {
		/* Nothing to wait for. Completing here rather than arming for the
		 * next tick announcement is what lets a deadline that has already
		 * passed stop its chain before any of it reaches an iodev.
		 */
		rtio_tq_claim(iodev_sqe);
		rtio_iodev_sqe_ok(iodev_sqe, 0);
		return;
	}

	key = k_spin_lock(&data->lock);

	if (rtio_tq_insert(data, iodev_sqe)) {
		/* New nearest deadline: re-arm, dropping the lock to retry if the
		 * handler is in flight on another CPU (SMP).
		 */
		while (!rtio_tq_try_arm(data)) {
			k_spin_unlock(&data->lock, key);
			key = k_spin_lock(&data->lock);
		}
	}

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Actively cancel a pending delay by removing it from the timeout queue
 *
 * The submission arrives as a handle because the canceling context does not own
 * it, so it is resolved under @p data->lock, which every path that completes a
 * pending delay also holds. A handle that no longer resolves belongs to an entry
 * that already completed and was recycled, and is a no-op.
 *
 * If the delay is still pending it is unlinked and the submission completed with
 * -ECANCELED. The shared timeout only needs re-arming when the removed entry was
 * the head (nearest deadline); cancelling any later delay leaves the armed
 * deadline unchanged. If the delay already expired (not in the list) this is a
 * no-op: its completion is already in flight. Once unlinked no other path can
 * complete the entry, so completion runs with the lock released, matching
 * rtio_tq_expired().
 */
static void rtio_timeout_iodev_cancel(const struct rtio_iodev *iodev, struct rtio *r,
				      rtio_sqe_handle_t handle)
{
	struct rtio_timeout_iodev_data *data = iodev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	/* Resolve under the lock: every path that completes a pending delay (the
	 * expiry handler and this hook) holds it, so a handle that still resolves
	 * here names an entry that cannot be completed and recycled underneath us.
	 */
	struct rtio_iodev_sqe *iodev_sqe = rtio_iodev_sqe_from_handle(r, handle);
	bool was_head;
	bool was_pending;

	if (iodev_sqe == NULL) {
		/* Already completed and recycled; nothing of ours left to abort. */
		k_spin_unlock(&data->lock, key);
		return;
	}

	was_head = (data->head == iodev_sqe);
	was_pending = rtio_tq_remove(data, iodev_sqe);

	if (was_pending) {
		rtio_tq_claim(iodev_sqe);
	}

	if (was_head) {
		/* We removed the nearest deadline, so the shared timeout must be
		 * re-armed to the new head. z_try_abort_timeout() (inside
		 * rtio_tq_try_arm) returns -EAGAIN if the expiry handler is running on
		 * another CPU and would deadlock on data->lock; the kernel's contract
		 * is to drop the lock so it can finish, then retry. On non-SMP this
		 * loop always runs exactly once.
		 */
		while (!rtio_tq_try_arm(data)) {
			k_spin_unlock(&data->lock, key);
			key = k_spin_lock(&data->lock);
		}
	}

	k_spin_unlock(&data->lock, key);

	if (was_pending) {
		rtio_iodev_sqe_err(iodev_sqe, -ECANCELED);
	}
}

/**
 * @brief Unlink a submission from the timeout queue, leaving it to the caller
 *
 * The counterpart to the cancel hook for a caller that owns the submission and
 * finishes it itself, which is the executor disarming a deadline whose chain has
 * ended. Removing it under the lock is also what keeps the expiry handler from
 * taking it afterwards, so the caller can tell that no action will be started
 * behind its back.
 */
void rtio_timeout_disarm(struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_timeout_iodev_data *data = iodev_sqe->sqe.iodev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	bool was_head = (data->head == iodev_sqe);

	(void)rtio_tq_remove(data, iodev_sqe);

	if (was_head) {
		/* See rtio_timeout_iodev_cancel() on the re-arm retry */
		while (!rtio_tq_try_arm(data)) {
			k_spin_unlock(&data->lock, key);
			key = k_spin_lock(&data->lock);
		}
	}

	k_spin_unlock(&data->lock, key);
}

static const struct rtio_iodev_api rtio_timeout_iodev_api = {
	.submit = rtio_timeout_iodev_submit,
	.cancel = rtio_timeout_iodev_cancel,
};

static struct rtio_timeout_iodev_data rtio_timeout_iodev_data_inst;

RTIO_IODEV_DEFINE(rtio_timeout_iodev, &rtio_timeout_iodev_api, &rtio_timeout_iodev_data_inst);
