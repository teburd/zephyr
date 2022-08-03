/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/interrupt_controller/dw_ace_v1x.h>

#include <cavs-idc.h>
#include <adsp_shim.h>

#ifdef CONFIG_SOC_SERIES_INTEL_ACE1X
#include <ace_v1x-regs.h>
#endif

/**
 * @file
 * @brief Intel Audio DSP Wall Clock Timer driver
 *
 * The Audio DSP on Intel SoC has a timer with one counter and two compare
 * registers that is external to the CPUs. This timer is accessible from
 * all available CPU cores and provides a synchronized timer under SMP.
 */

#define COMPARATOR_IDX  0 /* 0 or 1 */

#ifdef CONFIG_SOC_SERIES_INTEL_ACE1X
#define TIMER_IRQ MTL_IRQ_TO_ZEPHYR(MTL_INTL_TTS)
#else
#define TIMER_IRQ DSP_WCT_IRQ(COMPARATOR_IDX)
#endif

#define CYC_PER_TICK	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
			/ CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_CYC		0xFFFFFFFFUL
#define MAX_TICKS	((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 100000)

BUILD_ASSERT(MIN_DELAY < CYC_PER_TICK);
BUILD_ASSERT(COMPARATOR_IDX >= 0 && COMPARATOR_IDX <= 1);

#define WCTCS      (ADSP_SHIM_DSPWCTS)
#define COUNTER_HI (ADSP_SHIM_DSPWCH)
#define COUNTER_LO (ADSP_SHIM_DSPWCL)
#define COMPARE_HI (ADSP_SHIM_COMPARE_HI(COMPARATOR_IDX))
#define COMPARE_LO (ADSP_SHIM_COMPARE_LO(COMPARATOR_IDX))


static struct k_spinlock lock;
static uint64_t last_count;

#if defined(CONFIG_TEST)
const int32_t z_sys_timer_irq_for_test = TIMER_IRQ; /* See tests/kernel/context */
#endif


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

struct timer_event {
	enum timer_event_kind kind;
	int cpu;
	uint32_t ccount;
	uint64_t tcount;
	uint64_t tcompare;
	
	/* given only for timeout enter events */
	int64_t data;
};

#define TRACE_COUNT 10000
atomic_t timer_trace_idx = ATOMIC_INIT(-1);
struct timer_event timer_trace[TRACE_COUNT];

static uint64_t compare(void)
{
	uint32_t hi0, hi1, lo;

	do {
		hi0 = *COMPARE_HI;
		lo = *COMPARE_LO;
		hi1 = *COMPARE_HI;
	} while (hi0 != hi1);

	return (((uint64_t)hi0) << 32) | lo;
}

static uint64_t count(void)
{
	/* The count register is 64 bits, but we're a 32 bit CPU that
	 * can only read four bytes at a time, so a bit of care is
	 * needed to prevent racing against a wraparound of the low
	 * word.  Wrap the low read between two reads of the high word
	 * and make sure it didn't change.
	 */
	uint32_t hi0, hi1, lo;

	do {
		hi0 = *COUNTER_HI;
		lo = *COUNTER_LO;
		hi1 = *COUNTER_HI;
	} while (hi0 != hi1);

	return (((uint64_t)hi0) << 32) | lo;
}

static uint32_t count32(void)
{
	return *COUNTER_LO;
}


/**
 * The current index is always considered the write ptr
 *
 * Every write therefore needs to obtain the next index to write to
 */
static int32_t next_trace_idx() {
	uint32_t curr;

	do {
		curr = atomic_get(&timer_trace_idx);
	} while (!atomic_cas(&timer_trace_idx, curr, curr+1));

	return curr+1;
}

static inline void record_trace(enum timer_event_kind kind, int64_t data)
{
	uint32_t trace_idx = next_trace_idx();
	if (trace_idx >= TRACE_COUNT) {
		return;
	}

	__asm__ __volatile__("rsr %0,ccount":"=a" (timer_trace[trace_idx].ccount));
	timer_trace[trace_idx].kind = kind;
	timer_trace[trace_idx].cpu = arch_curr_cpu()->id;
	timer_trace[trace_idx].tcount = count();
	timer_trace[trace_idx].tcompare = compare();
	timer_trace[trace_idx].data = data;
}


static void set_compare(uint64_t time)
{
	record_trace(SET_COMPARE_ENTER, time);

	/* Disarm the comparator to prevent spurious triggers */
	*WCTCS &= ~DSP_WCT_CS_TA(COMPARATOR_IDX);

	*COMPARE_LO = (uint32_t)time;
	*COMPARE_HI = (uint32_t)(time >> 32);

	/* Arm the timer */
	*WCTCS |= DSP_WCT_CS_TA(COMPARATOR_IDX);
	
	record_trace(SET_COMPARE_EXIT, -1);
}

static void compare_isr(const void *arg)
{
	ARG_UNUSED(arg);
	uint64_t curr;
	uint32_t dticks;

	record_trace(COMPARE_ENTER, -1);

	k_spinlock_key_t key = k_spin_lock(&lock);

	record_trace(COMPARE_LOCKED, -1);

	curr = count();
	dticks = (uint32_t)((curr - last_count) / CYC_PER_TICK);

	/* Clear the triggered bit */
	*WCTCS |= DSP_WCT_CS_TT(COMPARATOR_IDX);

	last_count += dticks * CYC_PER_TICK;

#ifndef CONFIG_TICKLESS_KERNEL
	uint64_t next = last_count + CYC_PER_TICK;

	if ((int64_t)(next - curr) < MIN_DELAY) {
		next += CYC_PER_TICK;
	}
	set_compare(next);
#endif

	k_spin_unlock(&lock, key);

	record_trace(COMPARE_ANNOUNCE, dticks);

	sys_clock_announce(dticks);

	record_trace(COMPARE_EXIT, -1);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#ifdef CONFIG_TICKLESS_KERNEL
	record_trace(SET_TIMEOUT_ENTER, ticks);
	
	ticks = ticks == K_TICKS_FOREVER ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	record_trace(SET_TIMEOUT_CLAMPED, ticks);
	
	k_spinlock_key_t key = k_spin_lock(&lock);

	record_trace(SET_TIMEOUT_LOCKED, -1);

	uint64_t curr = count();
	uint64_t next;
	uint32_t adj, cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary */
	adj = (uint32_t)(curr - last_count) + (CYC_PER_TICK - 1);
	if (cyc <= MAX_CYC - adj) {
		cyc += adj;
	} else {
		cyc = MAX_CYC;
	}
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;
	next = last_count + cyc;

	if (((uint32_t)next - (uint32_t)curr) < MIN_DELAY) {
		next += CYC_PER_TICK;
	}

	set_compare(next);
	k_spin_unlock(&lock, key);
#endif
	record_trace(SET_TIMEOUT_EXIT, -1);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}
	record_trace(ELAPSED_ENTER, -1);

	k_spinlock_key_t key = k_spin_lock(&lock);

	record_trace(ELAPSED_LOCKED, -1);

	uint32_t ret = (count32() - (uint32_t)last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);

	record_trace(ELAPSED_EXIT, ret);
	return ret;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return count32();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return count();
}

/* Interrupt setup is partially-cpu-local state, so needs to be
 * repeated for each core when it starts.  Note that this conforms to
 * the Zephyr convention of sending timer interrupts to all cpus (for
 * the benefit of timeslicing).
 */
static void irq_init(void)
{
	int cpu = arch_curr_cpu()->id;

	/* These platforms have an extra layer of interrupt masking
	 * (for per-core control) above the interrupt controller.
	 * Drivers need to do that part.
	 */
#ifdef CONFIG_SOC_SERIES_INTEL_ACE1X
	MTL_DINT[cpu].ie[MTL_INTL_TTS] |= BIT(COMPARATOR_IDX + 1);
	*WCTCS |= ADSP_SHIM_DSPWCTCS_TTIE(COMPARATOR_IDX);
#else
	CAVS_INTCTRL[cpu].l2.clear = CAVS_L2_DWCT0;
#endif
	irq_enable(TIMER_IRQ);
}

void smp_timer_init(void)
{
	irq_init();
}

/* Runs on core 0 only */
static int sys_clock_driver_init(const struct device *dev)
{
	uint64_t curr = count();

	IRQ_CONNECT(TIMER_IRQ, 0, compare_isr, 0, 0);
	set_compare(curr + CYC_PER_TICK);
	last_count = curr;
	irq_init();
	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
