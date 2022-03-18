#include <stdlib.h>
#include <zephyr.h>

#define STACK_SIZE 2048
#define TIMER_PERIOD_US 1000
/* Low priority preemptible worker threads */
struct k_thread work_threads[CONFIG_MP_NUM_CPUS];
K_THREAD_STACK_ARRAY_DEFINE(work_stacks, CONFIG_MP_NUM_CPUS, STACK_SIZE);

/* Highest priority threads to handle timer events */
struct k_thread timer_threads[CONFIG_MP_NUM_CPUS];
K_THREAD_STACK_ARRAY_DEFINE(timer_stacks, CONFIG_MP_NUM_CPUS, STACK_SIZE);

struct k_sem timer_sems[CONFIG_MP_NUM_CPUS];

struct k_timer timers[CONFIG_MP_NUM_CPUS];

uint32_t worst_err[CONFIG_MP_NUM_CPUS];
uint64_t err_sum[CONFIG_MP_NUM_CPUS];
uint32_t err_count[CONFIG_MP_NUM_CPUS];

uint32_t ccount(void)
{
	uint32_t cyc;

	__asm__ volatile("rsr %0, CCOUNT" : "=r"(cyc));
	return cyc;
}


void work_fn(void *arg1, void *arg2, void *arg3)
{
	int cpu = (int)arg1;
	printk("ANDY %s:%d cpu%d (real %d)\n", __func__, __LINE__, cpu, arch_curr_cpu()->id);

	/* Just spin, coming up every once in a while to remark that
	 * we're running and report statistics
	 */
	while (true) {
		__ASSERT_NO_MSG(cpu == arch_curr_cpu()->id);

		uint32_t c0 = ccount();

		while(ccount() - c0 < 400000000) {}

		/* Note: strictly this needs synchronization because
		 * we can be preempted by timer_thread_fn which
		 * updates these values, but for stats this is good
		 * enough
		 */
		printk("work%d: worst %d avg %d (N = %d)\n",
		       cpu, worst_err[cpu], (int)(err_sum[cpu]/err_count[cpu]),
		       err_count[cpu]);
	}
}

void timer_thread_fn(void *arg1, void *arg2, void *arg3)
{
	printk("ANDY %s:%d\n", __func__, __LINE__);
	const int32_t tick_nominal = k_us_to_cyc_floor32(TIMER_PERIOD_US);
	int cpu = (int)arg1;
	uint32_t cyc, last_cyc = k_cycle_get_32();

	while (true) {
		__ASSERT_NO_MSG(cpu == arch_curr_cpu()->id);

		k_sem_take(&timer_sems[cpu], K_FOREVER);
		cyc = k_cycle_get_32();

		uint32_t err = abs((int32_t)(cyc - last_cyc) - tick_nominal);

		err_sum[cpu] += err;
		err_count[cpu] += 1;
		if (err > worst_err[cpu]) {
			worst_err[cpu] = err;
		}
		last_cyc = cyc;
	}
}

void timer_fn(struct k_timer *t)
{
	/* Note: this is the cpu for which we are delivering the
	 * notification, that will not in general be the same as the
	 * cpu this ISR is running on.
	 */
	int cpu = (int)(t - &timers[0]);

	k_sem_give(&timer_sems[cpu]);
}

void main(void)
{
	printk("initializing...\n");

	for (int i = 0; i < CONFIG_MP_NUM_CPUS; i++) {
		int work_prio = k_thread_priority_get(k_current_get()) + 1;
		k_timeout_t timeout = K_USEC(TIMER_PERIOD_US);

		k_sem_init(&timer_sems[i], 0, 999);
		k_timer_init(&timers[i], timer_fn, NULL);

		k_thread_create(&work_threads[i], work_stacks[i], STACK_SIZE,
				work_fn, (void *)i, NULL, NULL,
				work_prio, 0, K_FOREVER);
		k_thread_cpu_mask_clear(&work_threads[i]);
		k_thread_cpu_mask_enable(&work_threads[i], i);

		k_thread_create(&timer_threads[i], timer_stacks[i], STACK_SIZE,
				timer_thread_fn, (void *)i, NULL, NULL,
				K_HIGHEST_APPLICATION_THREAD_PRIO, 0, K_FOREVER);
		k_thread_cpu_mask_clear(&timer_threads[i]);
		k_thread_cpu_mask_enable(&timer_threads[i], i);

		k_timer_start(&timers[i], timeout, timeout);
		k_thread_start(&work_threads[i]);
		k_thread_start(&timer_threads[i]);
	}
}
