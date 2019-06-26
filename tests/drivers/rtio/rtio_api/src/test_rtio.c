/*
 * Copyright (c) 2019 Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <rtio.h>

RTIO_POOL_DEFINE(small_pool, 1024, 1);

void test_rtio_pool(void)
{
	struct rtio_block *block;
	struct rtio_block *block2;

	u32_t size = 1024 - sizeof(struct rtio_block);
	int res = rtio_block_alloc(&small_pool, &block, size, K_NO_WAIT);

	zassert_equal(res, 0, "Allocation should not fail");
	zassert_equal(block->size, size, "Size should be 1024");
	zassert_equal(block->len, 0, "Len should be zero");
	zassert_not_null(block->_data, "Data should not be NULL");
	res = rtio_block_alloc(&small_pool, &block2, size, K_NO_WAIT);

	zassert_equal(res, -ENOMEM, "Allocation should fail with no memory");

	rtio_block_free(block);

	res = rtio_block_alloc(&small_pool, &block2, size, K_NO_WAIT);

	zassert_equal(res, 0, "Allocation should not fail");
}

RTIO_POOL_DEFINE(pipe_pool, 1024, 1);

#define STACK_SIZE 512
K_THREAD_STACK_DEFINE(source_stack, STACK_SIZE);
static struct k_thread source_thread;
K_THREAD_STACK_DEFINE(sink_stack, STACK_SIZE);
static struct k_thread sink_thread;

RTIO_PIPE_DEFINE(pipe);

#define EXPECTED_VAL 5

void source_entry(void *pipev, void *p2, void *p3)
{
	struct rtio_pipe *pipe = (struct rtio_pipe *)pipev;
	struct rtio_block *block;
	int res = rtio_block_alloc(&pipe_pool, &block, sizeof(u32_t),
				   K_NO_WAIT);
	zassert_equal(res, 0, "Allocation should not fail");
	rtio_block_begin(block);
	rtio_block_push_u32_t(block, (u32_t)EXPECTED_VAL);
	rtio_block_end(block);
	rtio_pipe_push_block(pipe, block);
}

void sink_entry(void *pipev, void *p2, void *p3)
{
	struct rtio_pipe *pipe = (struct rtio_pipe *)pipev;
	struct rtio_block *block = rtio_pipe_pull_block(pipe, K_FOREVER);
	u32_t val = 0;

	zassert_equal(block->len, sizeof(u32_t),
		      "Block should contain one u32_t");
	zassert_equal(block->size, sizeof(u32_t),
		      "Block should be sized to hold one u32_t");
	zassert_equal(rtio_block_pull_u32_t(block, 0, &val), sizeof(u32_t),
		      "Block pull of one u32_t should succeed");
	zassert_equal(val, EXPECTED_VAL,
		      "Pulled value should match expected");
	rtio_block_free(block);
}

void test_rtio_pipe(void)
{
	k_tid_t source_tid = k_thread_create(&source_thread, source_stack,
					     STACK_SIZE, source_entry,
					     &pipe, NULL, NULL, 0, 0, 120);

	k_tid_t sink_tid = k_thread_create(&sink_thread, sink_stack, STACK_SIZE,
					   sink_entry, &pipe, NULL, NULL, 0, 0,
					   120);

	k_thread_start(source_tid);
	k_thread_start(sink_tid);
	k_yield();
	k_sleep(200);
}
