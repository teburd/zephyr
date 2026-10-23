/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/rtio.h>
#include <zephyr/kernel.h>

#include "rtio_internal.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rtio_executor, CONFIG_RTIO_LOG_LEVEL);

/* Deprecated pre-submit cancellation flag (RTIO_SQE_CANCELED), spelled out so
 * honoring it does not itself emit the deprecation warning. Remove once the
 * flag is gone.
 */
#define RTIO_SQE_CANCELED_LEGACY BIT(3)

/**
 * @brief Whether a submission is a deadline guarding the chain behind it
 */
static inline bool rtio_sqe_is_guard(const struct rtio_iodev_sqe *iodev_sqe)
{
	return IS_ENABLED(CONFIG_RTIO_OP_DEADLINE) && iodev_sqe->sqe.op == RTIO_OP_DEADLINE;
}

/**
 * @brief Stop the chain a guard covers
 *
 * Cancellation cascades forward one link at a time, so marking the earliest
 * submission that has not completed stops every submission behind it. A guarded
 * chain is walkable for its whole life: its members are not returned to the pool
 * as they complete, and the last of them links back to the guard.
 *
 * The member is handed to its iodev as a handle rather than a pointer because
 * this may run from the system clock announce context, where the entry could
 * complete concurrently; the iodev resolves the handle under its own lock.
 *
 * @param guard The deadline submission heading the chain
 */
static void rtio_guard_stop_chain(struct rtio_iodev_sqe *guard)
{
	struct rtio *r = guard->r;
	struct rtio_iodev_sqe *member = guard->next;

	while (member != NULL && member != guard) {
		if (!rtio_iodev_sqe_is_done(member)) {
			const struct rtio_iodev *iodev = member->sqe.iodev;
			rtio_sqe_handle_t handle = rtio_sqe_handle(r, &member->sqe);

			rtio_iodev_sqe_set_canceled(member);

			if (iodev != NULL && iodev->api->cancel != NULL) {
				iodev->api->cancel(iodev, r, handle);
			}
			return;
		}
		member = member->next;
	}
}

/**
 * @brief Release a guard and the chain it covered
 *
 * The one place a guarded chain is torn down: members are kept allocated as they
 * complete so the guard can still walk them, and the guard itself is kept until
 * the chain is over so the chain can always reach it. The verdict recorded on
 * the guard is reported last, once every member has reported its own result.
 *
 * @param guard The deadline submission
 */
static void rtio_guard_teardown(struct rtio_iodev_sqe *guard)
{
	struct rtio *r = guard->r;
	void *userdata = guard->sqe.userdata;
	uint32_t cqe_flags = rtio_cqe_compute_flags(guard);
	bool requires_response = FIELD_GET(RTIO_SQE_NO_RESPONSE, guard->sqe.flags) == 0;
	int result = rtio_guard_verdict(guard);
	struct rtio_iodev_sqe *member = guard->next;

	while (member != NULL && member != guard) {
		struct rtio_iodev_sqe *next = member->next;

		rtio_sqe_pool_free(r->sqe_pool, member);
		member = next;
	}
	rtio_sqe_pool_free(r->sqe_pool, guard);

	if (requires_response) {
		rtio_cqe_submit(r, result, userdata, cqe_flags);
	}
}

/**
 * @brief Handle the guard's own completion, reported by the timeout iodev
 *
 * A guard does not complete when its timer does: it records the verdict, stops
 * the chain and waits for it to unwind, so that its completion is the last word
 * on the work it covers. An expiry (@p is_ok) means the deadline elapsed, while
 * anything else means the guard was canceled, which stops the chain just the
 * same.
 *
 * The timeout iodev claimed the guard before handing it over, so a chain that
 * ends while this runs leaves the teardown here rather than freeing the guard
 * underneath it. Which also covers the chain ending from inside the stop below,
 * where a member's iodev aborts it on the spot.
 *
 * @param guard  The deadline submission
 * @param result Result the iodev reported
 * @param is_ok  Whether the iodev reported success
 */
static void rtio_guard_armed_done(struct rtio_iodev_sqe *guard, int result, bool is_ok)
{
	rtio_guard_set_verdict(guard, is_ok ? -ETIMEDOUT : result);

	rtio_guard_stop_chain(guard);

	if ((atomic_and(&guard->status, ~RTIO_SQE_STATUS_FINISHING) &
	     RTIO_SQE_STATUS_ENDED) != 0) {
		rtio_guard_teardown(guard);
	}
}

/**
 * @brief Finish a guarded chain that has ended
 *
 * Disarming under the timeout iodev's lock settles which side finishes the
 * guard: either the deadline is still pending, and taking it out of the queue
 * means no expiry or cancellation can be acted on afterwards, or it is gone and
 * the action that took it either has finished the guard already or is claimed
 * and will on its way out.
 *
 * @param guard The deadline submission heading the finished chain
 */
static void rtio_guard_chain_ended(struct rtio_iodev_sqe *guard)
{
	atomic_or(&guard->status, RTIO_SQE_STATUS_ENDED);

	rtio_timeout_disarm(guard);

	if ((atomic_get(&guard->status) & RTIO_SQE_STATUS_FINISHING) == 0) {
		rtio_guard_teardown(guard);
	}
}

/**
 * @brief Finish a chain whose last submission has completed
 *
 * @param next What follows the completed submission: the guard of a guarded
 *             chain, or NULL
 */
static void rtio_chain_done(struct rtio_iodev_sqe *next)
{
	if (next != NULL && rtio_sqe_is_guard(next)) {
		rtio_guard_chain_ended(next);
	}
}

/**
 * @brief Callback which completes an RTIO_AWAIT_OP handled by the executor
 *
 * The callback is triggered when the rtio_sqe tied to the RTIO_AWAIT_OP
 * is signaled by the user.
 *
 * @param iodev_sqe Submission to complete
 * @param userdata Additional data passed along
 */
static void rtio_executor_sqe_signaled(struct rtio_iodev_sqe *iodev_sqe, void *userdata)
{
	ARG_UNUSED(userdata);

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

/**
 * @brief Executor handled submissions
 */
static void rtio_executor_op(struct rtio_iodev_sqe *iodev_sqe, int last_result)
{
	const struct rtio_sqe *sqe = &iodev_sqe->sqe;

	switch (sqe->op) {
	case RTIO_OP_CALLBACK:
		sqe->callback.callback(iodev_sqe->r, sqe, last_result, sqe->callback.arg0);
		rtio_iodev_sqe_ok(iodev_sqe, 0);
		break;
	case RTIO_OP_AWAIT:
		rtio_iodev_sqe_await_signal(iodev_sqe, rtio_executor_sqe_signaled, NULL);
		break;
	default:
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
	}
}

static void rtio_guard_dispatch(struct rtio_iodev_sqe *guard);

/**
 * @brief Submit to an iodev a submission to work on
 *
 * Should be called by the executor when it wishes to submit work
 * to an iodev.
 *
 * @param iodev_sqe Submission to work on
 */
static inline void rtio_iodev_submit(struct rtio_iodev_sqe *iodev_sqe, int last_result)
{
	if (rtio_sqe_is_guard(iodev_sqe)) {
		rtio_guard_dispatch(iodev_sqe);
		return;
	}

	if (rtio_iodev_sqe_is_canceled(iodev_sqe)) {
		rtio_iodev_sqe_err(iodev_sqe, -ECANCELED);
		return;
	}

	/* No iodev means its an executor specific operation */
	if (iodev_sqe->sqe.iodev == NULL) {
		rtio_executor_op(iodev_sqe, last_result);
		return;
	}

	iodev_sqe->sqe.iodev->api->submit(iodev_sqe);
}

/**
 * @brief Arm a deadline and start the chain it guards
 *
 * A guard heads its chain but is not a step in it: holding the chain up until
 * the deadline elapsed would be the opposite of guarding it. So the timer is
 * armed and the chain started, and the two run alongside each other.
 *
 * Arming before starting the chain is what makes an already elapsed deadline
 * stop the work before it reaches an iodev: the expiry marks the chain head,
 * and the dispatch below refuses to hand a canceled submission to its iodev.
 *
 * @param guard The deadline submission to arm
 */
static void rtio_guard_dispatch(struct rtio_iodev_sqe *guard)
{
	struct rtio_iodev_sqe *chain = guard->next;

	__ASSERT((guard->sqe.flags & RTIO_SQE_CHAINED) != 0,
		 "A deadline must be chained to the submissions it guards");

	if (chain == NULL || chain == guard) {
		/* Nothing to guard, so nothing to arm either */
		rtio_guard_set_verdict(guard, -EINVAL);
		rtio_guard_teardown(guard);
		return;
	}

	if (rtio_iodev_sqe_is_canceled(guard)) {
		/* Canceled before it could be armed; the chain stops all the same */
		rtio_guard_set_verdict(guard, -ECANCELED);
		rtio_iodev_sqe_set_canceled(chain);
	} else {
		guard->sqe.iodev->api->submit(guard);
	}

	rtio_iodev_submit(chain, 0);
}

/**
 * @brief Submit operations in the queue to iodevs
 *
 * @param r RTIO context
 *
 * @retval 0 Always succeeds
 */
void rtio_executor_submit(struct rtio *r)
{
	struct mpsc_node *node = mpsc_pop(&r->sq);

	while (node != NULL) {
		struct rtio_iodev_sqe *iodev_sqe = CONTAINER_OF(node, struct rtio_iodev_sqe, q);

		/* Honor the deprecated pre-submit cancellation flag by
		 * transferring it into the status word.
		 */
		if ((iodev_sqe->sqe.flags & RTIO_SQE_CANCELED_LEGACY) != 0) {
			rtio_iodev_sqe_set_canceled(iodev_sqe);
		}

		iodev_sqe->r = r;

		struct rtio_iodev_sqe *curr = iodev_sqe, *next;
		/* The deadline covering the rest of the chain, usually its head */
		struct rtio_iodev_sqe *guard = rtio_sqe_is_guard(iodev_sqe) ? iodev_sqe : NULL;

		/* Link up transaction or queue list if needed */
		while (curr->sqe.flags & (RTIO_SQE_TRANSACTION | RTIO_SQE_CHAINED)) {
#ifdef CONFIG_ASSERT
			bool transaction = iodev_sqe->sqe.flags & RTIO_SQE_TRANSACTION;
			bool chained = iodev_sqe->sqe.flags & RTIO_SQE_CHAINED;
			bool multishot = iodev_sqe->sqe.flags & RTIO_SQE_MULTISHOT;

			__ASSERT((transaction ^ chained ^ multishot) &&
				 !(transaction && chained && multishot),
				 "Cannot have more than one of these flags"
				 " enabled: transaction, chained or multishot");
#endif
			node = mpsc_pop(&iodev_sqe->r->sq);

			__ASSERT(node != NULL,
				    "Expected a valid submission in the queue while in a transaction or chain");

			next = CONTAINER_OF(node, struct rtio_iodev_sqe, q);

			if ((next->sqe.flags & RTIO_SQE_CANCELED_LEGACY) != 0) {
				rtio_iodev_sqe_set_canceled(next);
			}

			/* Cancellation cascades down the chain like a failure */
			if (rtio_iodev_sqe_is_canceled(curr)) {
				rtio_iodev_sqe_set_canceled(next);
			}

			if (guard != NULL) {
				__ASSERT(!rtio_sqe_is_guard(next),
					 "A deadline cannot guard another deadline");
				/* A guarded member is released with the rest of its
				 * chain rather than as it completes, so the guard can
				 * walk the chain it still owns when its deadline
				 * elapses.
				 */
				atomic_or(&next->status, RTIO_SQE_STATUS_GUARDED);
			} else if (rtio_sqe_is_guard(next)) {
				/* A deadline covers what follows it, which is the whole
				 * chain when it heads one and the remainder otherwise.
				 */
				guard = next;
			}

			curr->next = next;
			curr = next;
			curr->r = r;

			__ASSERT(
				curr != NULL,
				"Expected a valid sqe following transaction or chain flag");
		}

		/* The last member of a guarded chain links back to the guard, so the
		 * guard is reachable from the chain as it advances and the whole set
		 * can be released when it ends. Unguarded chains end in a NULL link as
		 * before. Chain and transaction walks stop on the flags rather than on
		 * a NULL link, so they are unaffected either way.
		 */
		curr->next = guard;
		curr->r = r;

		rtio_iodev_submit(iodev_sqe, 0);

		node = mpsc_pop(&r->sq);
	}
}

/**
 * @brief Release a submission that has completed
 *
 * A member of a guarded chain is kept allocated and only marked done: the whole
 * chain is released together when it ends, so that the guard can walk the chain
 * it still owns for as long as its deadline can elapse.
 *
 * @param r         RTIO context that owns @p iodev_sqe
 * @param iodev_sqe The completed submission
 */
static inline void rtio_executor_sqe_release(struct rtio *r, struct rtio_iodev_sqe *iodev_sqe)
{
	if (rtio_iodev_sqe_is_guarded(iodev_sqe)) {
		rtio_iodev_sqe_set_done(iodev_sqe);
	} else {
		rtio_sqe_pool_free(r->sqe_pool, iodev_sqe);
	}
}

/**
 * @brief Handle common logic when :c:macro:`RTIO_SQE_MULTISHOT` is set
 *
 * @param[in] iodev_sqe IODEV SQE that's being marked as finished.
 * @param[in] result The result of the latest request iteration
 * @param[in] is_ok Whether or not the SQE's result was successful
 */
static inline void rtio_executor_handle_multishot(struct rtio_iodev_sqe *iodev_sqe,
						  int result, bool is_ok)
{
	struct rtio *r = iodev_sqe->r;
	const bool is_canceled = rtio_iodev_sqe_is_canceled(iodev_sqe);
	const bool uses_mempool = FIELD_GET(RTIO_SQE_MEMPOOL_BUFFER, iodev_sqe->sqe.flags) == 1;
	const bool requires_response = FIELD_GET(RTIO_SQE_NO_RESPONSE, iodev_sqe->sqe.flags) == 0;
	uint32_t cqe_flags = rtio_cqe_compute_flags(iodev_sqe);
	void *userdata = iodev_sqe->sqe.userdata;
	struct rtio_iodev_sqe *chain_next = NULL;

	/** We're releasing reasources when erroring as an error handling scheme of multi-shot
	 * submissions by requiring to stop re-submitting if something goes wrong. Let the
	 * application decide what's best for handling the corresponding error: whether
	 * re-submitting, rebooting or anything else.
	 */
	if (is_canceled || !is_ok) {
		/* Say the stream was stopped, unless there is a real error to report */
		if (is_canceled && is_ok) {
			result = -ECANCELED;
		}

		LOG_DBG("Releasing memory @%p size=%u", (void *)iodev_sqe->rt.rx_bind.buf,
			iodev_sqe->rt.rx_bind.buf_len);
		rtio_release_buffer(r, iodev_sqe->rt.rx_bind.buf, iodev_sqe->rt.rx_bind.buf_len);

		/* Read the link before releasing, which may recycle the entry */
		chain_next = iodev_sqe->next;
		rtio_executor_sqe_release(r, iodev_sqe);

		/* The blocks are back in the pool, so do not hand them out again */
		cqe_flags = 0;
	}

	if (requires_response) {
		rtio_cqe_submit(r, result, userdata, cqe_flags);
	}

	/* A stream only ever ends a chain, so stopping it ends the chain too */
	if (is_canceled || !is_ok) {
		rtio_chain_done(chain_next);
	}

	/* resubmit directly to the iodev if the sqe has not stopped */
	if (is_ok && !is_canceled) {
		/* Request was not canceled, re-arm it */
		if (iodev_sqe->sqe.op == RTIO_OP_RX && uses_mempool) {
			/* Reset the buffer info so the next request can get a new one */
			iodev_sqe->rt.rx_bind.buf = NULL;
			iodev_sqe->rt.rx_bind.buf_len = 0;
		}

		rtio_iodev_submit(iodev_sqe, 0);
	}
}

/**
 * @brief Handle common logic one-shot items
 *
 * @param[in] iodev_sqe IODEV SQE that's being marked as finished.
 * @param[in] result The result of the latest request iteration
 * @param[in] is_ok Whether or not the SQE's result was successful
 */
static inline void rtio_executor_handle_oneshot(struct rtio_iodev_sqe *iodev_sqe,
						int last_result, bool is_ok)
{
	const bool is_canceled = rtio_iodev_sqe_is_canceled(iodev_sqe);
	struct rtio_iodev_sqe *curr = iodev_sqe;
	struct rtio *r = iodev_sqe->r;
	uint32_t sqe_flags;
	int result = last_result;

	/** Single-shot items may be linked as transactions or be chained together.
	 * Untangle the set of SQEs and act accordingly on each one.
	 */
	do {
		void *userdata = curr->sqe.userdata;
		uint32_t cqe_flags = rtio_cqe_compute_flags(curr);
		struct rtio_iodev_sqe *next = rtio_iodev_sqe_next(curr);

		sqe_flags = curr->sqe.flags;

		if (FIELD_GET(RTIO_SQE_NO_RESPONSE, sqe_flags) == 0) {
			/* Generate a result back to the client if need be.*/
			rtio_cqe_submit(r, result, userdata, cqe_flags);
		}

		rtio_executor_sqe_release(r, curr);
		curr = next;

		if (!is_ok) {
			/* This is an error path, so cancel any chained SQEs */
			result = -ECANCELED;
		}
	} while (FIELD_GET(RTIO_SQE_TRANSACTION, sqe_flags) == 1);

	/* curr should now be the last sqe in the transaction if that is what completed */
	if (FIELD_GET(RTIO_SQE_CHAINED, sqe_flags) == 1) {
		/* A chain cascades failure, so a canceled link stops the ones behind
		 * it. Cancellation only ever marks the submission its handle names,
		 * and the mark is carried forward one link at a time here rather than
		 * by walking the chain from the canceling context, where the entries
		 * are not owned and may be completed and recycled mid-walk. The
		 * submission below does not reach the iodev once marked: it completes
		 * the link with -ECANCELED, which frees it and brings us back here for
		 * the link after it.
		 */
		if (is_canceled) {
			rtio_iodev_sqe_set_canceled(curr);
		}
		rtio_iodev_submit(curr, last_result);
	} else {
		/* End of the chain; curr is its guard, if it had one */
		rtio_chain_done(curr);
	}
}

static inline void rtio_executor_done(struct rtio_iodev_sqe *iodev_sqe, int result, bool is_ok)
{
	const bool is_multishot = FIELD_GET(RTIO_SQE_MULTISHOT, iodev_sqe->sqe.flags) == 1;

	if (rtio_sqe_is_guard(iodev_sqe)) {
		rtio_guard_armed_done(iodev_sqe, result, is_ok);
	} else if (is_multishot) {
		rtio_executor_handle_multishot(iodev_sqe, result, is_ok);
	} else {
		rtio_executor_handle_oneshot(iodev_sqe, result, is_ok);
	}
}

/**
 * @brief Callback from an iodev describing success
 */
void rtio_executor_ok(struct rtio_iodev_sqe *iodev_sqe, int result)
{
	rtio_executor_done(iodev_sqe, result, true);
}

/**
 * @brief Callback from an iodev describing error
 *
 * Some assumptions are made and should have been validated on rtio_submit
 * - a sqe marked as chained or transaction has a next sqe
 * - a sqe is marked either chained or transaction but not both
 */
void rtio_executor_err(struct rtio_iodev_sqe *iodev_sqe, int result)
{
	rtio_executor_done(iodev_sqe, result, false);
}
