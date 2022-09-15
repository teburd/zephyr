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


enum timer_event_kind {
	COMPARE_ENTER,
	COMPARE_LOCKED,
	COMPARE_ANNOUNCE,
	COMPARE_EXIT,
	ELAPSED_ENTER,
	ELAPSED_LOCKED,
	ELAPSED_EXIT,
	SET_TIMEOUT_ENTER,
	SET_TIMEOUT_CLAMPED,
	SET_TIMEOUT_LOCKED,
	SET_TIMEOUT_EXIT,
	SET_COMPARE_ENTER,
	SET_COMPARE_EXIT,
};

const char * timer_event_str(enum timer_event_kind kind) {
	switch(kind) {
		case COMPARE_ENTER: 
			return "COMPARE_ENTER";
		case COMPARE_LOCKED:
			return "COMPARE_LOCKED"; 
		case COMPARE_ANNOUNCE:
			return "COMPARE_ANNOUNCE"; 
		case COMPARE_EXIT:
			return "COMPARE_EXIT";
		case ELAPSED_ENTER:
			return "ELAPSED_ENTER";
		case ELAPSED_LOCKED:
			return "ELAPSED_LOCKED";
		case ELAPSED_EXIT:
			return "ELAPSED_EXIT";
		case SET_TIMEOUT_ENTER:
			return "SET_TIMEOUT_ENTER";
		case SET_TIMEOUT_CLAMPED:
			return "SET_TIMEOUT_CLAMPED";
		case SET_TIMEOUT_LOCKED:
			return "SET_TIMEOUT_LOCKED";
		case SET_TIMEOUT_EXIT:
			return "SET_TIMEOUT_EXIT";
		case SET_COMPARE_ENTER:
			return "SET_COMPARE_ENTER";
		case SET_COMPARE_EXIT:
			return "SET_COMPARE_EXIT";
	}
	return "UNKNOWN";
}

struct timer_event {
	enum timer_event_kind kind;
	int cpu;
	uint32_t ccount;
	uint64_t tcount;
	uint64_t tcompare;
	int64_t data;
};

#define TRACE_COUNT 8192
#define TRACE_IDX_MASK (8192-1)
extern atomic_t timer_trace_idx;
extern struct timer_event timer_trace[TRACE_COUNT];

extern uint32_t last_ipc_isr;
extern uint32_t ipc_idx;
extern uint32_t ipc_diffs[8192];

uint32_t periodic_idx = 0;
uint32_t periodic_diffs[SAMPLES];

uint32_t absolute_misses = 0;
uint32_t absolute_idx = 0;
uint32_t absolute_diffs[SAMPLES];

void ipc_awaiting_thread(void *p1, void *p2, void *p3)
{
	struct k_sem *sem = p1;
	
	while (true) {
		/* Setup the next IPC interrupt */
		intel_adsp_ipc_send_message_sync(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_RETURN_MSG, ipc_idx, K_FOREVER);

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

#define IPC_INFO_BUILD		BIT(0)
#define IPC_INFO_LOCKS		BIT(1)
#define IPC_INFO_LOCKSV		BIT(2)
#define IPC_INFO_GDB		BIT(3)
#define IPC_INFO_D3_PERSISTENT	BIT(4)

/* ipc4 notification msg */
enum sof_ipc4_notification_type {
	SOF_IPC4_NOTIFY_PHRASE_DETECTED		= 4,
	SOF_IPC4_NOTIFY_RESOURCE_EVENT		= 5,
	SOF_IPC4_NOTIFY_LOG_BUFFER_STATUS	= 6,
	SOF_IPC4_NOTIFY_TIMESTAMP_CAPTURED	= 7,
	SOF_IPC4_NOTIFY_FW_READY		= 8,
	SOF_IPC4_FW_AUD_CLASS_RESULT		= 9,
	SOF_IPC4_EXCEPTION_CAUGHT		= 10,
	SOF_IPC4_MODULE_NOTIFICATION		= 12,
	SOF_IPC4_UAOL_RSVD_		= 13,
	SOF_IPC4_PROBE_DATA_AVAILABLE		= 14,
	SOF_IPC4_WATCHDOG_TIMEOUT		= 15,
	SOF_IPC4_MANAGEMENT_SERVICE		= 16,
};

#define SOF_IPC4_GLB_NOTIFY_DIR_MASK		BIT(29)
#define SOF_IPC4_REPLY_STATUS_MASK		0xFFFFFF
#define SOF_IPC4_GLB_NOTIFY_TYPE_SHIFT		16
#define SOF_IPC4_GLB_NOTIFY_MSG_TYPE_SHIFT		24

#define SOF_IPC4_FW_READY \
		(((SOF_IPC4_NOTIFY_FW_READY) << (SOF_IPC4_GLB_NOTIFY_TYPE_SHIFT)) |\
		((SOF_IPC4_GLB_NOTIFICATION) << (SOF_IPC4_GLB_NOTIFY_MSG_TYPE_SHIFT)))



#define GLB_TYPE_SHIFT			28
#define GLB_TYPE_MASK			(0xfUL << GLB_TYPE_SHIFT)
#define GLB_TYPE(x)			((x) << GLB_TYPE_SHIFT)

#define IPC_FW_READY			GLB_TYPE(0x7U)

struct ipc_hdr {
	uint32_t size;			/**< size of structure */
} __attribute__((packed, aligned(4)));

struct ipc_cmd_hdr {
	uint32_t size;			/**< size of structure */
	uint32_t cmd;			/**< SOF_IPC_GLB_ + cmd */
} __attribute__((packed, aligned(4)));


/* fw version */
struct ipc_fw_version {
	struct ipc_hdr hdr;
	uint16_t major;
	uint16_t minor;
	uint16_t micro;
	uint16_t build;
	uint8_t date[12];
	uint8_t time[10];
	uint8_t tag[6];
	uint32_t abi_version;
	/** used to check FW and ldc file compatibility, reproducible value (ABI3.17) */
	uint32_t src_hash;

	/* reserved for future use */
	uint32_t reserved[3];
} __attribute__((packed, aligned(4)));

/* FW ready Message - sent by firmware when boot has completed */
struct ipc_fw_ready {
	struct ipc_cmd_hdr hdr;
	uint32_t dspbox_offset;	 /* dsp initiated IPC mailbox */
	uint32_t hostbox_offset; /* host initiated IPC mailbox */
	uint32_t dspbox_size;
	uint32_t hostbox_size;
	struct ipc_fw_version version;

	/* Miscellaneous flags */
	uint64_t flags;

	/* reserved for future use */
	uint32_t reserved[4];
} __attribute__((packed, aligned(4)));


static const struct ipc_fw_ready ready = {
	.hdr = {
		.cmd = IPC_FW_READY,
		.size = sizeof(struct ipc_fw_ready),
	},
	.version = {
		.hdr.size = sizeof(struct ipc_fw_version),
		.micro = 0,
		.minor = 0,
		.major = 0,
		.build = -1,
		.date = "dtermin.\0",
		.time = "fwready.\0",
		.tag = SOF_TAG,
		.abi_version = SOF_ABI_VERSION,
		.src_hash = SOF_SRC_HASH,
	},
	.flags = IPC_INFO_BUILD,
};

void main(void)
{
	uint32_t trace_reader_idx = 0;

	/* Copy the "fw ready" message to the mailbox */
	void *mbox = (void *)(uintptr_t)(HP_SRAM_WIN0_BASE + 0x4000);
	memcpy(mbox, &ready, sizeof(ready));

	/* Notify the kernel we are ready */
	intel_adsp_ipc_send_message_sync(INTEL_ADSP_IPC_HOST_DEV, SOF_IPC4_FW_READY, 0, K_MSEC(1000));
	
	start_cycle = k_cycle_get_64();
	last_ipc_isr = start_cycle;
	last_periodic_isr = start_cycle;
	last_absolute_isr = start_cycle;
	
	/* Schedule timers */
	k_timer_start(&periodic_tm, K_MSEC(1), K_MSEC(1));
	absolute_scheduled_tick = k_uptime_ticks() + ABSOLUTE_TIMER_TICKS;
	k_timer_start(&absolute_tm, K_TIMEOUT_ABS_CYC(absolute_scheduled_tick), K_NO_WAIT);
	

	while (true) {
		//update_stats(ipc_idx, ipc_diffs, 8192, &ipc_stats);
		update_stats(periodic_idx, periodic_diffs, SAMPLES, &periodic_stats);
		update_stats(absolute_idx, absolute_diffs, SAMPLES, &absolute_stats);

	uint64_t kern_tick = k_uptime_ticks();
		uint64_t abs_sched_tick = absolute_scheduled_tick;
		int64_t diff_ticks = (int64_t)abs_sched_tick-(int64_t)kern_tick;

		/* Update the screen without interrupts */	
		unsigned int key = irq_lock();
		printk("===TRACE BEGIN===\n");
		printk("ADSP IPC & Timer Saturation: ipc isrs %u, periodic isrs %u, absolute isrs %u, misses %u, ticks %u\n", ipc_idx, periodic_idx, absolute_idx, absolute_misses, ABSOLUTE_TIMER_TICKS);
		printk("IPC ISR Statistics: min %12.3f, max %12.3f, mean %12.3f, variance %12.3f, stddev %12.3f\n", ipc_stats.min, ipc_stats.max, ipc_stats.mean, ipc_stats.variance, ipc_stats.stddev);
		printk("Periodic ISR Statistics: min %12.3f, max %12.3f, mean %12.3f, variance %12.3f, stddev %12.3f\n", periodic_stats.min, periodic_stats.max, periodic_stats.mean, periodic_stats.variance, periodic_stats.stddev);
		printk("Absolute ISR Statistics: min %12.3f, max %12.3f, mean %12.3f, variance %12.3f, stddev %12.3f\n", absolute_stats.min, absolute_stats.max, absolute_stats.mean, absolute_stats.variance, absolute_stats.stddev);
		printk("Current: %llu (ticks), Next Absolute: %llu (ticks) Diff: %lld (ticks)\n", 
			kern_tick, absolute_scheduled_tick, diff_ticks);
		
		

		printk("\n");
		
		const uint32_t display = 32;
		/* Dump time trace data */
		uint32_t trace_writer_idx = atomic_get(&timer_trace_idx);
		uint32_t trace_start_idx = trace_writer_idx < display ? trace_reader_idx 
			: MAX(trace_reader_idx, trace_writer_idx - display);
		for (uint32_t i = trace_start_idx; i < trace_writer_idx; i++) {
			printk("TTRACE[%d]: event %s, cpu %d, ccount %d, tcount %llu, tcompare %llu, data %lld\n",
				i, 
				timer_event_str(timer_trace[i & TRACE_IDX_MASK].kind),
				timer_trace[i & TRACE_IDX_MASK].cpu,
				timer_trace[i & TRACE_IDX_MASK].ccount,
				timer_trace[i & TRACE_IDX_MASK].tcount,
				timer_trace[i & TRACE_IDX_MASK].tcompare,
				timer_trace[i & TRACE_IDX_MASK].data);
			trace_reader_idx = i;
		}
		printk("===TRACE END===\n\n");

		irq_unlock(key);


		/* Timer #3 */
		k_msleep(500);
	}
}
