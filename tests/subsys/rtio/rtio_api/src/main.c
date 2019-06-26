/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ztest_assert.h"
#include <errno.h>
#include <ztest.h>
#include <kernel.h>
#include <sys/atomic.h>
#include <rtio/rtio_fifo.h>
#include <rtio/rtio.h>

/*
 * @brief Produce and Consume a single uint32_t in the same execution context
 *
 * @see rtio_fifo_acquire(), rtio_fifo_produce(), rtio_fifo_consume(), rtio_fifo_release()
 *
 * @ingroup rtio_tests
 */
void test_produce_consume_size1(void)
{
	RTIO_FIFO_DEFINE(ezfifo, uint32_t, 1);

	const uint32_t magic = 43219876;

	uint32_t *acq = rtio_fifo_acquire(ezfifo);

	zassert_not_null(acq, "Acquire should succeed");

	*acq = magic;

	uint32_t *acq2 = rtio_fifo_acquire(ezfifo);

	zassert_is_null(acq2, "Acquire should fail");

	uint32_t *cons = rtio_fifo_consume(ezfifo);

	zassert_is_null(cons, "Consume should fail");

	zassert_equal(rtio_fifo_consumable(ezfifo), 0, "Consumables should be 0");

	rtio_fifo_produce(ezfifo);

	zassert_equal(rtio_fifo_consumable(ezfifo), 1, "Consumables should be 1");

	uint32_t *cons2 = rtio_fifo_consume(ezfifo);

	zassert_equal(rtio_fifo_consumable(ezfifo), 0, "Consumables should be 0");

	zassert_not_null(cons2, "Consume should not fail");
	zassert_equal(*cons2, magic, "Consume value should equal magic");

	uint32_t *cons3 = rtio_fifo_consume(ezfifo);

	zassert_is_null(cons3, "Consume should fail");


	uint32_t *acq3 = rtio_fifo_acquire(ezfifo);

	zassert_is_null(acq3, "Acquire should not succeed");

	rtio_fifo_release(ezfifo);

	uint32_t *acq4 = rtio_fifo_acquire(ezfifo);

	zassert_not_null(acq4, "Acquire should succeed");
}

/*&*
 * @brief Produce and Consume 3 items at a time in a fifo of size 4 to validate masking
 * and wrap around reads/writes.
 *
 * @see rtio_fifo_acquire(), rtio_fifo_produce(), rtio_fifo_consume(), rtio_fifo_release()
 *
 * @ingroup rtio_tests
 */
void test_produce_consume_wrap_around(void)
{
	RTIO_FIFO_DEFINE(ezfifo, uint32_t, 4);

	for (int i = 0; i < 10; i++) {
		zassert_equal(rtio_fifo_consumable(ezfifo), 0, "Consumables should be 0");
		for (int j = 0; j < 3; j++) {
			uint32_t *entry = rtio_fifo_acquire(ezfifo);
			zassert_not_null(entry, "Acquire should succeed");
			*entry = i * 3 + j;
			rtio_fifo_produce(ezfifo);
		}
		zassert_equal(rtio_fifo_consumable(ezfifo), 3, "Consumables should be 3");

		for (int k = 0; k < 3; k++) {
			uint32_t *entry = rtio_fifo_consume(ezfifo);
			zassert_not_null(entry, "Consume should succeed");
			zassert_equal(*entry, i * 3 + k, "Consume value should equal i*3+k");
			rtio_fifo_release(ezfifo);
		}

		zassert_equal(rtio_fifo_consumable(ezfifo), 0, "Consumables should be 0");

	}
}

/**
 * @brief Ensure that integer wraps continue to work.
 *
 * Done by setting all values to UINT32_MAX - 2 and writing and reading enough
 * to ensure integer wraps occur.
 */
void test_int_wrap_around(void)
{
	RTIO_FIFO_DEFINE(ezfifo, uint32_t, 4);
	ezfifo->_fifo.in = ATOMIC_INIT(UINT32_MAX - 2);
	ezfifo->_fifo.out = ATOMIC_INIT(UINT32_MAX - 2);

	for (int j = 0; j < 3; j++) {
		uint32_t *entry = rtio_fifo_acquire(ezfifo);

		zassert_not_null(entry, "Acquire should succeed");
		*entry = j;
		rtio_fifo_produce(ezfifo);
	}

	zassert_equal(atomic_get(&ezfifo->_fifo.in), UINT32_MAX + 1, "Fifo in should wrap");

	for (int k = 0; k < 3; k++) {
		uint32_t *entry = rtio_fifo_consume(ezfifo);

		zassert_not_null(entry, "Consume should succeed");
		zassert_equal(*entry, k, "Consume value should equal i*3+k");
		rtio_fifo_release(ezfifo);
	}

	zassert_equal(atomic_get(&ezfifo->_fifo.out), UINT32_MAX + 1, "Fifo out should wrap");
}

#define MAX_RETRIES 5
#define SMP_ITERATIONS 100

RTIO_FIFO_DEFINE(spsc, uint32_t, 4);

static void t1_consume(void *p1, void *p2, void *p3)
{
	struct rtio_fifo_spsc *ezfifo = p1;
	uint32_t retries = 0;
	uint32_t *val = NULL;

	for (int i = 0; i < SMP_ITERATIONS; i++) {
		val = NULL;
		retries = 0;
		while (val == NULL && retries < MAX_RETRIES) {
			val = rtio_fifo_consume(ezfifo);
			retries++;
		}
		if (val != NULL) {
			rtio_fifo_release(ezfifo);
		} else {
			printk("consumer yield\n");
			k_yield();
		}
	}
}

static void t2_produce(void *p1, void *p2, void *p3)
{
	struct rtio_fifo_spsc *ezfifo = p1;
	uint32_t retries = 0;
	uint32_t *val = NULL;

	for (int i = 0; i < SMP_ITERATIONS; i++) {
		val = NULL;
		retries = 0;
		printk("producer acquiring\n");
		while (val == NULL && retries < MAX_RETRIES) {
			val = rtio_fifo_acquire(ezfifo);
			retries++;
		}
		if (val != NULL) {
			*val = SMP_ITERATIONS;
			rtio_fifo_produce(ezfifo);
		} else {
			printk("producer yield\n");
			k_yield();
		}
	}
}

#define STACK_SIZE (384 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define THREADS_NUM 2

struct thread_info {
	k_tid_t tid;
	int executed;
	int priority;
	int cpu_id;
};
static ZTEST_BMEM volatile struct thread_info tinfo[THREADS_NUM];
static struct k_thread tthread[THREADS_NUM];
static K_THREAD_STACK_ARRAY_DEFINE(tstack, THREADS_NUM, STACK_SIZE);


/**
 * @brief Test that the producer and consumer are indeed thread safe
 *
 * This can and should be validated on SMP machines where incoherent
 * memory could cause issues.
 */
void test_spsc_threaded(void)
{

	tinfo[0].tid =
		k_thread_create(&tthread[0], tstack[0], STACK_SIZE,
				(k_thread_entry_t)t1_consume,
				spsc, NULL, NULL,
				K_PRIO_PREEMPT(5),
				K_INHERIT_PERMS, K_NO_WAIT);
	tinfo[1].tid =
		k_thread_create(&tthread[1], tstack[1], STACK_SIZE,
				(k_thread_entry_t)t2_produce,
				spsc, NULL, NULL,
				K_PRIO_PREEMPT(5),
				K_INHERIT_PERMS, K_NO_WAIT);

	k_thread_join(tinfo[1].tid, K_FOREVER);
	k_thread_join(tinfo[0].tid, K_FOREVER);
}



static void rtio_iodev_test_submit(const struct rtio_sqe *sqe,
				   struct rtio *r,
				   uint32_t flags);

static const struct rtio_iodev_api rtio_iodev_test_api = {
	.submit = rtio_iodev_test_submit
};

/* Test device submission queue event */
struct rtio_iodev_sqe {
	struct rtio_sqe sqe;
	struct rtio *r;
	uint32_t flags;
	struct k_sem done;
};

/**
 * @brief Test device submission queue
 *
 * This is used for reifying the member of the rtio_iodev_test struct
 */
struct rtio_iodev_sq {
	struct rtio_fifo _fifo;
	struct rtio_iodev_sqe buffer[];
};

/*
 * @brief A simple asynchronous testable iodev with a queue and thread
 */
struct rtio_iodev_test {
	/* io device struct */
	struct rtio_iodev iodev;

	/* Request queue, should not be necessary for real devices where ISRs,
	 * IP FIFOs and possibly DMA provide the asynchronous request exectuion.
	 *
	 * In some cases, an asynchronous work q item perhaps might be needed
	 * to poll some registers for completion rather than waiting on
	 * interrupts.
	 *
	 * In all cases its up to the device in collaboration with the executor.
	 */
	struct rtio_iodev_sq *sq;

	/* Semaphore for completions, makes tests cleaner */
	struct k_sem cqe_sem;
};

#define IODEV_TEST_DEFINE(name, sz)                                    \
	RTIO_FIFO_DEFINE(_iodev_sq_##name, struct rtio_iodev_sqe, sz); \
	struct rtio_iodev_test name = {                                \
		.iodev = { .api = &rtio_iodev_test_api },              \
		.sq = (struct rtio_iodev_sq *const)_iodev_sq_##name    \
	}

static void rtio_iodev_test_execute(struct rtio_iodev_test *iodev, const struct rtio_sqe *sqe,
				    struct rtio *r, uint32_t flags)
{
	struct rtio_cqe *cqe = rtio_fifo_acquire(r->cq);

	if (cqe == NULL) {
		atomic_add(&r->xcqcnt, (atomic_val_t)1);
		goto out;
	}

	if (sqe->op == RTIO_OP_NOP) {
		cqe->result = 0;
		cqe->userdata = sqe->userdata;
	} else {
		cqe->result = -EINVAL;
		cqe->userdata = sqe->userdata;
	}

	rtio_fifo_produce(r->cq);

out:
	return;
}

volatile bool iodev_join;
K_THREAD_STACK_DEFINE(iodev_stack, 256);
struct k_thread iodev_thread;

static void rtio_iodev_exec(void *p0, void *p1, void *p2)
{
	struct rtio_iodev_test * const iodev = (struct rtio_iodev_test * const)p0;

	while (!iodev_join) {
		struct rtio_iodev_sqe *iodev_sqe = rtio_fifo_consume(iodev->sq);

		if (iodev_sqe != NULL) {
			rtio_iodev_test_execute(iodev, &iodev_sqe->sqe, iodev_sqe->r,
						iodev_sqe->flags);
			k_sem_give(&iodev_sqe->done);
			rtio_fifo_release(iodev->sq);
		}

		/* Simulate a task that consumes time */
		k_sleep(K_MSEC(10));

		k_sem_give(&iodev->cqe_sem);
	}
}

static void rtio_iodev_test_submit(
	const struct rtio_sqe *sqe,
	struct rtio *r,
	uint32_t flags)
{
	struct rtio_iodev_test *iodev = (struct rtio_iodev_test *)sqe->iodev;
	TC_PRINT("acquire: acquire %u, in %lu, consume %u, out %lu, mask %x\n",
		 iodev->sq->_fifo.acquire, atomic_get(&iodev->sq->_fifo.in),
		 iodev->sq->_fifo.consume, atomic_get(&iodev->sq->_fifo.out),
		 iodev->sq->_fifo.mask);

	/* Emulate non-blocking requests with a queue
	 * and k_thread
	 */
	struct k_sem *sem;
	struct rtio_iodev_sqe *iodev_sqe = rtio_fifo_acquire(iodev->sq);

	TC_PRINT("iodev_sqe is %p\n", iodev_sqe);

	if (iodev_sqe != NULL) {
		memcpy(&iodev_sqe->sqe, sqe, sizeof(struct rtio_sqe));
		iodev_sqe->r = r;
		iodev_sqe->flags = flags;
		k_sem_init(&iodev_sqe->done, 0, 1);
		sem = &iodev_sqe->done;
		rtio_fifo_produce(iodev->sq);
		TC_PRINT("produced: acquire %u, in %lu, consume %u, out %lu, mask %x\n",
			 iodev->sq->_fifo.acquire, atomic_get(&iodev->sq->_fifo.in),
			 iodev->sq->_fifo.consume, atomic_get(&iodev->sq->_fifo.out),
			 iodev->sq->_fifo.mask);

		if (flags & RTIO_SUBMIT_SYNC) {
			k_sem_take(sem, K_FOREVER);
		}
	} else {
		TC_PRINT("iodev_sqe is NULL (%p), return -EWOULDBLOCK\n", iodev_sqe);
		struct rtio_cqe *cqe = rtio_fifo_acquire(r->cq);

		if (cqe == NULL) {
			atomic_add(&r->xcqcnt, (atomic_val_t)1);
			return;
		}

		cqe->userdata = sqe->userdata;
		cqe->result = -EWOULDBLOCK;

		rtio_fifo_produce(r->cq);
	}
}

#include <rtio/rtio_executor_inplace.h>
RTIO_EXECUTOR_INPLACE_DEFINE(inplace_exec);
IODEV_TEST_DEFINE(iodev_test, 4);
RTIO_DEFINE(r, inplace_exec, 4, 4);


/**
 * @brief Test the basics of the RTIO API
 *
 * Ensures that we can setup an RTIO context, enqueue a request, and receive
 * a completion event.
 */
void test_rtio_simple(void)
{
	int res;
	uint32_t userdata[2] = {0, 1};
	/*  uint8_t wbuf[8], rbuf[8]; */
	struct rtio_sqe *sqe;
	struct rtio_cqe *cqe;

	rtio_fifo_reset(r.sq);
	rtio_fifo_reset(r.cq);
	k_sem_init(&iodev_test.cqe_sem, 0, 4);

	iodev_join = false;
	k_thread_create(&iodev_thread, iodev_stack,
			K_THREAD_STACK_SIZEOF(iodev_stack),
			rtio_iodev_exec,
			&iodev_test, NULL, NULL,
			5, 0, K_NO_WAIT);

	sqe = rtio_fifo_acquire(r.sq);
	zassert_not_null(sqe, "Expected a valid sqe");
	rtio_sqe_prep_nop(sqe, (struct rtio_iodev *)&iodev_test, &userdata[0]);
	rtio_fifo_produce(r.sq);

	res = rtio_submit(&r, 1);
	zassert_ok(res, "Should return ok from rtio_execute");

	cqe = rtio_fifo_consume(r.cq);
	zassert_not_null(cqe, "Expected a valid cqe");
	zassert_ok(cqe->result, "Result should be ok");
	zassert_equal_ptr(cqe->userdata, &userdata[0], "Expected userdata back");
	rtio_fifo_release(r.cq);

	iodev_join = true;
	k_thread_join(&iodev_thread, K_FOREVER);
}

/**
 * @brief Test chained requests
 *
 * Ensures that we can setup an RTIO context, enqueue a chained requests,
 * and receive completion events in the correct order given the chained
 * flag.
 */
void test_rtio_chain(void)
{
	int res;
	uint32_t userdata[4] = {0, 1, 2, 3};
	struct rtio_sqe *sqe;
	struct rtio_cqe *cqe;

	rtio_fifo_reset(r.sq);
	rtio_fifo_reset(r.cq);
	k_sem_init(&iodev_test.cqe_sem, 0, 4);

	iodev_join = false;
	k_thread_create(&iodev_thread, iodev_stack,
			K_THREAD_STACK_SIZEOF(iodev_stack),
			rtio_iodev_exec,
			&iodev_test, NULL, NULL,
			5, 0, K_NO_WAIT);

	for (int i = 0; i < 4; i++) {
		sqe = rtio_fifo_acquire(r.sq);
		zassert_not_null(sqe, "Expected a valid sqe");
		rtio_sqe_prep_nop(sqe, (struct rtio_iodev *)&iodev_test, &userdata[i]);
		sqe->flags |= RTIO_SQE_CHAINED;
		rtio_fifo_produce(r.sq);
	}
	zassert_equal(rtio_fifo_consumable(r.sq), 4, "Should have 4 pending ops");

	res = rtio_submit(&r, 4);
	zassert_ok(res, "Should return ok from rtio_execute");
	zassert_equal(rtio_fifo_consumable(r.cq), 4, "Should have 4 pending completions");

	for (int i = 0; i < 4; i++) {
		TC_PRINT("consume %d\n", i);
		cqe = rtio_fifo_consume(r.cq);
		zassert_not_null(cqe, "Expected a valid cqe");
		zassert_ok(cqe->result, "Result should be ok");
		zassert_equal_ptr(cqe->userdata, &userdata[i], "Expected in order completions");
		rtio_fifo_release(r.cq);
	}


	iodev_join = true;
	k_thread_join(&iodev_thread, K_FOREVER);
}

/**
 * @brief Test multiple asynchronous chains against one iodev
 */
void test_rtio_multiple_chains(void)
{
	int res;
	uint32_t userdata[4] = {0, 1, 2, 3};
	struct rtio_sqe *sqe;
	struct rtio_cqe *cqe;

	rtio_fifo_reset(r.sq);
	rtio_fifo_reset(r.cq);
	k_sem_init(&iodev_test.cqe_sem, 0, 4);

	iodev_join = false;
	k_thread_create(&iodev_thread, iodev_stack,
			K_THREAD_STACK_SIZEOF(iodev_stack),
			rtio_iodev_exec,
			&iodev_test, NULL, NULL,
			5, 0, K_NO_WAIT);

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			sqe = rtio_fifo_acquire(r.sq);
			zassert_not_null(sqe, "Expected a valid sqe");
			rtio_sqe_prep_nop(sqe, (struct rtio_iodev *)&iodev_test,
					  (void *)userdata[i*2 + j]);
			if (j == 0) {
				sqe->flags |= RTIO_SQE_CHAINED;
			} else {
				sqe->flags |= 0;
			}
			rtio_fifo_produce(r.sq);
		}
	}
	zassert_equal(rtio_fifo_consumable(r.sq), 4, "Should have 4 pending ops");

	res = rtio_submit(&r, 0);
	zassert_ok(res, "Should return ok from rtio_execute");

	for (int i = 0; i < 4; i++) {
		k_sem_take(&iodev_test.cqe_sem, K_MSEC(100));
	}

	bool seen[4] = { 0 };

	for (int i = 0; i < 4; i++) {
		cqe = rtio_fifo_consume(r.cq);
		zassert_not_null(cqe, "Expected a valid cqe");
		TC_PRINT("result %d, would block is %d, inval is %d\n",
			 cqe->result, -EWOULDBLOCK, -EINVAL);
		zassert_ok(cqe->result, "Result should be ok");
		seen[(uint32_t)cqe->userdata] = true;
		if (seen[1]) {
			zassert_true(seen[0], "Should see 0 before 1");
		}
		if (seen[3]) {
			zassert_true(seen[2], "Should see 2 before 3");
		}
		rtio_fifo_release(r.cq);
	}


	iodev_join = true;
	k_thread_join(&iodev_thread, K_FOREVER);
}

/**
 * @brief Test multiple asynchronous submission requests
 */



void test_main(void)
{
	ztest_test_suite(rtio_fifo_test,
			 ztest_1cpu_unit_test(test_produce_consume_size1),
			 ztest_1cpu_unit_test(test_produce_consume_wrap_around),
			 ztest_1cpu_unit_test(test_int_wrap_around),
			 ztest_unit_test(test_spsc_threaded),
			 ztest_unit_test(test_rtio_simple),
			 ztest_unit_test(test_rtio_chain),
			 ztest_unit_test(test_rtio_multiple_chains)
			 );

	ztest_run_test_suite(rtio_fifo_test);
}
