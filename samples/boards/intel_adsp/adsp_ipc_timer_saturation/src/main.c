/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <intel_adsp_ipc.h>
#include <cavstool.h>
#include <stdint.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>

K_SEM_DEFINE(ipc_sem, 0, K_SEM_MAX_LIMIT);
K_SEM_DEFINE(periodic_sem, 0, K_SEM_MAX_LIMIT);
K_SEM_DEFINE(absolute_sem, 0, K_SEM_MAX_LIMIT);

#define IPC_BUSY 990
#define PERIODIC_BUSY 990
#define ABSOLUTE_BUSY 990

#define SAMPLES  (1<<16)
#define IDX_MASK ((1<<16) - 1)

uint64_t start_cycle;

uint32_t ipc_idx = 0;
uint32_t ipc_diffs[SAMPLES];

uint32_t periodic_idx = 0;
uint32_t periodic_diffs[SAMPLES];

uint32_t absolute_misses = 0;
uint32_t absolute_idx = 0;
uint32_t absolute_diffs[SAMPLES];

void ipc_awaiting_thread(void *p1, void *p2, void *p3)
{
	struct k_sem *sem = p1;
	
	while (true) {
		k_sem_take(sem, K_FOREVER);

		/* Setup the next IPC interrupt */
		intel_adsp_ipc_send_message(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_RETURN_MSG, ipc_idx);

		k_busy_wait(IPC_BUSY);
	}
}

void periodic_awaiting_thread(void *p1, void *p2, void *p3)
{
	struct k_sem *sem = p1;
	
	while (true) {
		k_sem_take(sem, K_FOREVER);
		k_busy_wait(PERIODIC_BUSY);
	}
}

void absolute_awaiting_thread(void *p1, void *p2, void *p3)
{
	struct k_sem *sem = p1;
	
	while (true) {
		k_sem_take(sem, K_FOREVER);
		k_busy_wait(ABSOLUTE_BUSY);
	}
}


K_THREAD_DEFINE(ipc_work, 1024, ipc_awaiting_thread, &ipc_sem, NULL, NULL,
	K_PRIO_PREEMPT(3), K_ESSENTIAL, 0);
K_THREAD_DEFINE(periodic_work, 1024, periodic_awaiting_thread, &periodic_sem, NULL, NULL,
	K_PRIO_PREEMPT(2), K_ESSENTIAL, 0);
K_THREAD_DEFINE(absolute_work, 1024, absolute_awaiting_thread, &absolute_sem, NULL, NULL,
	K_PRIO_PREEMPT(1), K_ESSENTIAL, 0);

uint32_t last_ipc_isr;

void ipc_done_cb(const struct device *dev, void *arg)
{
	uint32_t now = k_cycle_get_32();
	
	ipc_diffs[ipc_idx & IDX_MASK] = now - last_ipc_isr;
	last_ipc_isr = now;
	ipc_idx++;

	/* Notify the worker thread */
	k_sem_give(&ipc_sem);
}

uint32_t last_periodic_isr;

void timer_periodic_cb(struct k_timer *tm)
{
	uint32_t now = k_cycle_get_32();
	
	/* Note the current time */
	periodic_diffs[periodic_idx & IDX_MASK] = now - last_periodic_isr;
	last_periodic_isr = now;
	periodic_idx++;
	
	/* Notify the worker thread */
	k_sem_give(&periodic_sem);
}

uint64_t absolute_scheduled_tick;

#define ABSOLUTE_TIMER_TICKS (CONFIG_SYS_CLOCK_TICKS_PER_SEC/1000)

uint64_t last_absolute_isr;

void timer_absolute_cb(struct k_timer *tm) {
 	uint64_t now = k_cycle_get_64();

	absolute_diffs[absolute_idx & IDX_MASK] = now - last_absolute_isr;
	last_absolute_isr = now;
	absolute_idx++;

	/* Setup the next absolute timer */
	/* this doesn't work? confusing */
	/* uint32_t next = absolute_scheduled_cycle + ABSOLUTE_TIMER_CYCLES; */
	uint64_t next = absolute_scheduled_tick + ABSOLUTE_TIMER_TICKS;

	/* next - now will be "negative" (large positive, much larger than ABSOLUTE_TIMER_CYCLES)
	 * if next is in the past.
	 */
	while (next < k_uptime_ticks()) {
		next += ABSOLUTE_TIMER_TICKS;
		absolute_misses++;
	}
	absolute_scheduled_tick = next;
	k_timeout_t abs_timeout = K_TIMEOUT_ABS_TICKS(next);
	
	k_timer_start(tm, abs_timeout, K_NO_WAIT);

	/* Notify the worker thread */
	k_sem_give(&absolute_sem);
}

/* Timer #1 */
K_TIMER_DEFINE(periodic_tm, timer_periodic_cb, NULL);

/* Timer #2 */
K_TIMER_DEFINE(absolute_tm, timer_absolute_cb, NULL);

/* Constants for computing a histogram with percentiles for latencies */
const double percentiles[] = {99.99, 99.9, 99.0, 95.0, 90.0, 75.0, 50.0};

/* Time offset buckets */
const double buckets[] = {-1024.0, -512.0, -256.0, -128.0, -64.0, -32.0, -16.0, -8.0, -4.0, -2.0, -1.0, 0.0,
	1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0}; 

struct statistics {
	double min;
	double max;
	double mean;
	double variance;
	double stddev;
};

static struct statistics ipc_stats;

static struct statistics periodic_stats;
static double periodic_histogram[ARRAY_SIZE(buckets)];

static struct statistics absolute_stats;
static double absolute_histogram[ARRAY_SIZE(buckets)];

double cycles_to_us(double cycles)
{
	return 1000000.0 * (cycles / (double)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
}

static inline void update_stats(uint32_t local_idx, uint32_t *vals, uint32_t vals_len, struct statistics *stats)
{
	uint32_t mask = vals_len - 1;
	stats->min = DBL_MAX;
	stats->max = -DBL_MAX;
	stats->mean = 0.0;
	stats->variance = 0.0;
	stats->stddev = 0.0;
	
	if(local_idx < 4) {
		return;
	}

	uint32_t window_size = 1024;
	if (local_idx < 1025) {
		window_size = local_idx - 2;
	}
	uint32_t samples = 0;
	double sum = 0.0;
	for (uint32_t idx = local_idx-1; idx > local_idx-1-window_size; idx--) {
		double diff = cycles_to_us(vals[idx & mask]);
		sum += diff;
		stats->min = MIN(diff, stats->min);
		stats->max = MAX(diff, stats->max);
		samples++;
	}
	stats->mean = sum/(double)samples;
	for (uint32_t idx = local_idx-1; idx > local_idx-1-window_size; idx--) {
		double diff = cycles_to_us(vals[idx & mask]);
		double mean_diff = diff - stats->mean;
	double mean_diff_sq = mean_diff*mean_diff;
		stats->variance += mean_diff_sq;
	}
	stats->variance = stats->variance/(double)samples;
	stats->stddev = sqrtf(stats->variance);
}

void main(void)
{
	start_cycle = k_cycle_get_64();
	last_ipc_isr = start_cycle;
	last_periodic_isr = start_cycle;
	last_absolute_isr = start_cycle;
	
	/* Set ipc handler */
	intel_adsp_ipc_set_done_handler(INTEL_ADSP_IPC_HOST_DEV, ipc_done_cb, &ipc_sem);

	/* Schedule timers */
	k_timer_start(&periodic_tm, K_MSEC(1), K_MSEC(1));

	absolute_scheduled_tick = k_uptime_ticks() + ABSOLUTE_TIMER_TICKS;
	k_timer_start(&absolute_tm, K_TIMEOUT_ABS_CYC(absolute_scheduled_tick), K_NO_WAIT);
	
	/* Send command to host for a *random duration* IPC command */
	intel_adsp_ipc_send_message(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_RETURN_MSG, ipc_idx);

	while (true) {
		update_stats(ipc_idx, ipc_diffs, SAMPLES, &ipc_stats);
		update_stats(periodic_idx, periodic_diffs, SAMPLES, &periodic_stats);
		update_stats(absolute_idx, absolute_diffs, SAMPLES, &absolute_stats);

		uint64_t kern_tick = k_uptime_ticks();
		uint64_t abs_sched_tick = absolute_scheduled_tick;
		int64_t diff_ticks = (int64_t)abs_sched_tick-(int64_t)kern_tick;

		/* Update the screen without interrupts */	
		unsigned int key = irq_lock();
		printk("\033[2J\033[1;1H");

		printk("ADSP IPC & Timer Saturation: ipc isrs %u, periodic isrs %u, absolute isrs %u, misses %u, ticks %u\n", ipc_idx, periodic_idx, absolute_idx, absolute_misses, ABSOLUTE_TIMER_TICKS);
		printk("IPC ISR Statistics: min %12.3f, max %12.3f, mean %12.3f, variance %12.3f, stddev %12.3f\n", ipc_stats.min, ipc_stats.max, ipc_stats.mean, ipc_stats.variance, ipc_stats.stddev);
		printk("Periodic ISR Statistics: min %12.3f, max %12.3f, mean %12.3f, variance %12.3f, stddev %12.3f\n", periodic_stats.min, periodic_stats.max, periodic_stats.mean, periodic_stats.variance, periodic_stats.stddev);
		printk("Absolute ISR Statistics: min %12.3f, max %12.3f, mean %12.3f, variance %12.3f, stddev %12.3f\n", absolute_stats.min, absolute_stats.max, absolute_stats.mean, absolute_stats.variance, absolute_stats.stddev);
		printk("Current: %llu (ticks), Next Absolute: %llu (ticks) Diff: %lld (ticks)\n", 
			kern_tick, absolute_scheduled_tick, diff_ticks);
		irq_unlock(key);


		/* Timer #3 */
		k_msleep(500);
	}
}
