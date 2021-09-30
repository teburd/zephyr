/*
 * Copyright (c) 2019 Tom Burdick <tom.burdick@electromatic.us>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <drivers/rtio.h>

RTIO_SLAB_ALLOCATOR_DEFINE(slab_allocator, 1024, 1, 4);

void test_rtio_slab_allocator(void)
{
	struct rtio_block *block;
	struct rtio_block *block2;

	uint32_t size = 1024;
	int res = rtio_block_alloc(slab_allocator, &block, size, K_NO_WAIT);

	zassert_equal(res, 0, "Allocation should not fail");
	zassert_equal(block->buf.size, size, "Size should be 1024");
	zassert_equal(block->buf.len, 0, "Len should be zero");
	zassert_not_null(block->buf.data, "Data should not be NULL");
	res = rtio_block_alloc(slab_allocator, &block2, size, K_NO_WAIT);

	zassert_equal(res, -ENOMEM, "Allocation should fail with no memory");

	rtio_block_free(slab_allocator, block);

	res = rtio_block_alloc(slab_allocator, &block2, size, K_NO_WAIT);

	zassert_equal(res, 0, "Allocation should not fail");

	rtio_block_free(slab_allocator, block2);
}

RTIO_SLAB_ALLOCATOR_DEFINE(src_sink_allocator, 1024, 1, 4);

#define STACK_SIZE 512
K_THREAD_STACK_DEFINE(source_stack, STACK_SIZE);
static struct k_thread source_thread;
K_THREAD_STACK_DEFINE(sink_stack, STACK_SIZE);
static struct k_thread sink_thread;

K_FIFO_DEFINE(block_fifo);

#define EXPECTED_VAL 5

void source_entry(void *fifov, void *p2, void *p3)
{
	struct k_fifo *fifo = (struct k_fifo *)fifov;
	struct rtio_block *block;
	int res = rtio_block_alloc(src_sink_allocator, &block, sizeof(uint32_t),
				   K_NO_WAIT);
	zassert_equal(res, 0, "Allocation should not fail");
	rtio_block_begin(block);
	rtio_block_add_le32(block, (uint32_t)EXPECTED_VAL);
	rtio_block_end(block);
	k_fifo_put(fifo, block);
}

void sink_entry(void *fifov, void *p2, void *p3)
{
	struct k_fifo *fifo = (struct k_fifo *)fifov;
	struct rtio_block *block = k_fifo_get(fifo, K_FOREVER);
	uint32_t val = 0;

	zassert_true(block->buf.len >= sizeof(uint32_t),
		      "Block should contain at least one uint32_t");
	zassert_true(block->buf.size >= sizeof(uint32_t),
		      "Block should be sized to hold at least one uint32_t");
	val = (uint32_t)rtio_block_pull_le32(block);
	zassert_equal(val, EXPECTED_VAL,
		      "Pulled value should match expected");
	rtio_block_free(src_sink_allocator, block);
}

void test_rtio_source_sink(void)
{
	k_tid_t source_tid = k_thread_create(&source_thread, source_stack, STACK_SIZE,
					     source_entry, &block_fifo, NULL, NULL,
					     K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	k_tid_t sink_tid = k_thread_create(&sink_thread, sink_stack, STACK_SIZE,
					   sink_entry, &block_fifo, NULL, NULL,
					   K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	k_thread_start(source_tid);
	k_thread_start(sink_tid);
	k_yield();
	k_thread_join(source_tid, K_FOREVER);
	k_thread_join(sink_tid, K_FOREVER);
}
