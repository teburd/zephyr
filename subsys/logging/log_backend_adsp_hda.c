/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/xtensa/cache.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_output_dict.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/logging/log_backend_adsp_hda.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>

/**
 * @brief HDA Log Backend
 * 
 * The HDA log backend works as a pipeline of tasks.
 * 
 * 1. Messages are formatted into a format buffer
 * 2. The format buffer is copied into the dma ring buffer
 * 3. The DMA controller is notified of bytes written to the dma ring buffer
 * 4. A hook is called to notify the other end of the line of available bytes
 * 5. When hook is done, those bytes become available again
 *    in the DMA buffer.
 *
 * Not every log flush results in a DMA flush and Hook call,
 * as those require hardware to respond and that may take some time.
 *
 * Instead things are batched up until a watermark is met or maximum
 * latency threshold has been met.
 */

//#define DBG(msg)
#define DBG(msg) printk("[%p %d %d %d %llu] %s\n", arch_curr_cpu()->current, arch_curr_cpu()->id, k_is_in_isr(), __LINE__, k_cycle_get_64(), msg)

static uint32_t log_format_current = CONFIG_LOG_BACKEND_ADSP_HDA_OUTPUT_DEFAULT;
static const struct device *const hda_log_dev =
	DEVICE_DT_GET(DT_NODELABEL(hda_host_in));
static uint32_t hda_log_chan;

/*
 * HDA requires 128 byte aligned data and 128 byte aligned transfers.
 */
#define ALIGNMENT DMA_BUF_ALIGNMENT(DT_NODELABEL(hda_host_in))
static __aligned(ALIGNMENT) uint8_t hda_log_buf[CONFIG_LOG_BACKEND_ADSP_HDA_SIZE];
static struct k_timer hda_log_timer;
static adsp_hda_log_hook_t hook;
//static struct k_spinlock hda_log_fmt_lock;
static struct k_spinlock hda_log_out_lock;
//static struct k_spinlock hda_log_flush_lock;
//K_SEM_DEFINE(hda_log_out_lock, 0, 1);

/* Simple power of 2 wrapping mask */
#define HDA_LOG_BUF_MASK (CONFIG_LOG_BACKEND_ADSP_HDA_SIZE - 1)

/* atomic bit flags for state */
#define HDA_LOG_DMA_READY 0
#define HDA_LOG_PANIC_MODE 1
static atomic_t hda_log_flags;

/* A forever increasing write_idx, should be masked with HDA_LOG_BUF_MASK when accessing hda_log_buf */
static volatile uint32_t write_idx;

/* Available bytes that are not pending anything in the pipe */
static volatile uint32_t available_bytes = CONFIG_LOG_BACKEND_ADSP_HDA_SIZE;

/* Bytes pending DMA flush */
static volatile uint32_t pending_dma_bytes;

/* Bytes pending Hook call */
static volatile uint32_t pending_hook_bytes;

/**
 * @brief When padding is enabled insert '\0' characters to align the buffer up
 */
static void hda_log_pad(void)
{
#ifdef CONFIG_LOG_BACKEND_ADSP_HDA_PADDING
	uint32_t next128, padding;
	uint32_t nearest128 = pending_dma_bytes & ~((128) - 1);

	/* No need to pad if we are already 128 byte aligned */
	if (nearest128 == pending_dma_bytes || pending_dma_bytes == 0) {
		return;
	}

	next128 = nearest128 + 128;
	padding = next128 - pending_dma_bytes;
	
	//printk("Log pad available %u, pending dma %u, pending hook %u\n", available_bytes, pending_dma_bytes, pending_hook_bytes);
	
	/* In order to pad the buffer we need to know there's free space */
	__ASSERT(available_bytes > padding, "Expected to always have bytes to pad up to the next 128");

	for (int i = 0; i < padding; i++) {
		hda_log_buf[write_idx & HDA_LOG_BUF_MASK] = '\0';
		write_idx++;
	}

	available_bytes -= padding;
	pending_dma_bytes += padding;

	nearest128 = pending_dma_bytes & ~((128) - 1);
	__ASSERT(pending_dma_bytes == nearest128,
		"Expected after padding pending bytes to be 128 byte aligned");
#endif
}

/**
 * @brief Write pending_dma_bytes to the HDA stream
 */
static void hda_log_dma_flush(void)
{
	uint32_t nearest128;
	struct dma_status status;
	
	//printk("Log dma flush available %u, pending dma %u, pending hook %u\n", available_bytes, pending_dma_bytes, pending_hook_bytes);

	/* Nothing to do */
	if (pending_dma_bytes == 0) {
		return;
	}

	nearest128 = pending_dma_bytes & ~((128) - 1);

	/* Nothing to do */
	if (nearest128 == 0) {
		return;
	}

	/* Without kernel coherence we need to ensure the buffer is flushed
	 * so the DMA hardware may pick it up from SRAM
	 */
#if !(IS_ENABLED(CONFIG_KERNEL_COHERENCE))
	z_xtensa_cache_flush(hda_log_buf, CONFIG_LOG_BACKEND_ADSP_HDA_SIZE);
#endif

	dma_reload(hda_log_dev, hda_log_chan, 0, 0, nearest128);

	/* Wait for the DMA hardware to catch up */
	do {
		__ASSERT(dma_get_status(hda_log_dev, hda_log_chan, &status) == 0,
			"DMA get status failed unexpectedly");

	} while (status.read_position != status.write_position);

	/* The bytes now move to the next step in the pipe */
	pending_dma_bytes -= nearest128;
	pending_hook_bytes += nearest128;
}

/**
 * @brief Write the pending_hook_bytes to the hook
 */
static void hda_log_hook(void) {
	if (hook == NULL || pending_hook_bytes == 0) {
		return;
	}
	
	//printk("Log hook available %u, pending dma %u, pending hook %u\n", available_bytes, pending_dma_bytes, pending_hook_bytes);

	/* Hook must only return once bytes are done being dealt with */
	hook(pending_hook_bytes);
	
	available_bytes += pending_hook_bytes;
	pending_hook_bytes = 0;
}

static int hda_log_out(uint8_t *data, size_t length, void *ctx)
{
	//DBG("out lock start");
	k_spinlock_key_t key = k_spin_lock(&hda_log_out_lock);
	//k_sem_take(&hda_log_out_lock, K_FOREVER);
	//DBG("out lock taken");

	__ASSERT( length < available_bytes, "Log buffer overflowed");

	/* Copy over the formatted message to the dma buffer */
	for (uint32_t i = 0; i < length; i++) {
		hda_log_buf[write_idx & HDA_LOG_BUF_MASK] = data[i];
		write_idx++;
	}
	available_bytes -= length;
	pending_dma_bytes += length;

	bool panic_mode_set = atomic_test_bit(&hda_log_flags, HDA_LOG_PANIC_MODE);
	
	/* If we've hit the watermark or are in panic mode push data out */
	if (available_bytes < CONFIG_LOG_BACKEND_ADSP_HDA_SIZE/2 || panic_mode_set) {

		//DBG("pad start");
		hda_log_pad();
		//DBG("pad end");

		//DBG("flush lock start");
		//k_spinlock_key_t flush_key = k_spin_lock(&hda_log_flush_lock);
		//DBG("flush lock end");
		//k_spin_unlock(&hda_log_out_lock, key);
		//DBG("out lock freed");

		if (atomic_test_bit(&hda_log_flags, HDA_LOG_DMA_READY)) {
			//DBG("dma flush start");
			hda_log_dma_flush();
			//DBG("dma flush end");
		}

		

		//DBG("hook start");
		hda_log_hook();
		//DBG("hook end");

		//k_spin_unlock(&hda_log_out_lock, key);
		//k_spin_unlock(&hda_log_flush_lock, flush_key);
	} else {

		//k_spin_unlock(&hda_log_out_lock, key);
		//k_spin_unlock(&hda_log_out_lock, key);
		//DBG("out lock freed, no flush");
}
	//k_sem_give(&hda_log_out_lock);
	
	k_spin_unlock(&hda_log_out_lock, key);
	return length;
}

K_SEM_DEFINE(periodic, 0, K_SEM_MAX_LIMIT);

static void hda_log_flusher(void *p1, void *p2, void *p3) 
{

	while(true) {
		k_sem_take(&periodic, K_FOREVER);
	
		DBG("periodic lock take");
		k_spinlock_key_t key = k_spin_lock(&hda_log_out_lock);
		DBG("periodic lock taken");
		/* Always try to pad */	
		hda_log_pad();

		/* Always write bytes to the DMA controller */
		if (atomic_test_bit(&hda_log_flags, HDA_LOG_DMA_READY)) {
			hda_log_dma_flush();
		}

		hda_log_hook();

		k_spin_unlock(&hda_log_out_lock, key);
		DBG("periodic lock released");	
	}	
}

K_THREAD_DEFINE(hda_log_thread, 512, hda_log_flusher,
	NULL, NULL, NULL,
	K_PRIO_COOP(1),  0, 0);
	
/**
 * 128 bytes is the smallest transferrable size for HDA so use that
 * and encompass almost all log lines in the formatter before flushing
 * and memcpy'ing to the HDA buffer.
 */
#define LOG_BUF_SIZE 512
static uint8_t log_buf[LOG_BUF_SIZE];
LOG_OUTPUT_DEFINE(log_output_adsp_hda, hda_log_out, log_buf, LOG_BUF_SIZE);

static void hda_log_periodic(struct k_timer *tm)
{
	ARG_UNUSED(tm);

	k_sem_give(&periodic);

}

struct k_spinlock log_out_lock;

static inline void dropped(const struct log_backend *const backend,
			   uint32_t cnt)
{
	ARG_UNUSED(backend);

	if (IS_ENABLED(CONFIG_LOG_DICTIONARY_SUPPORT)) {
		log_dict_output_dropped_process(&log_output_adsp_hda, cnt);
	} else {
		log_output_dropped_process(&log_output_adsp_hda, cnt);
	}
}

static void panic(struct log_backend const *const backend)
{
	ARG_UNUSED(backend);


	/* will immediately flush all future writes once set */
	atomic_set_bit(&hda_log_flags, HDA_LOG_PANIC_MODE);

	/* flushes the log queue */
	log_backend_std_panic(&log_output_adsp_hda);
}

static int format_set(const struct log_backend *const backend, uint32_t log_type)
{
	ARG_UNUSED(backend);

	log_format_current = log_type;

	return 0;
}

static void process(const struct log_backend *const backend,
		union log_msg_generic *msg)
{
	ARG_UNUSED(backend);
	
	uint32_t flags = log_backend_std_get_flags();

	log_format_func_t log_output_func = log_format_func_t_get(log_format_current);

	log_output_func(&log_output_adsp_hda, &msg->log, flags);

}

/**
 * Lazily initialized, while the DMA may not be setup we continue
 * to buffer log messages untilt he buffer is full.
 */
static void init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);

	available_bytes = CONFIG_LOG_BACKEND_ADSP_HDA_SIZE;
	pending_dma_bytes = 0;
	pending_hook_bytes = 0;
}

const struct log_backend_api log_backend_adsp_hda_api = {
	.process = process,
	.dropped = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ? NULL : dropped,
	.panic = panic,
	.format_set = format_set,
	.init = init,
};

LOG_BACKEND_DEFINE(log_backend_adsp_hda, log_backend_adsp_hda_api, true);

void adsp_hda_log_init(adsp_hda_log_hook_t fn, uint32_t channel)
{
	hook = fn;

	int res;

	__ASSERT(device_is_ready(hda_log_dev), "DMA device is not ready");

	hda_log_chan = dma_request_channel(hda_log_dev, &channel);
	__ASSERT(hda_log_chan >= 0, "No valid DMA channel");
	__ASSERT(hda_log_chan == channel, "Not requested channel");

	/* configure channel */
	struct dma_block_config hda_log_dma_blk_cfg = {
		.block_size = CONFIG_LOG_BACKEND_ADSP_HDA_SIZE,
		.source_address = (uint32_t)(uintptr_t)&hda_log_buf,
	};

	struct dma_config hda_log_dma_cfg = {
		.channel_direction = MEMORY_TO_HOST,
		.block_count = 1,
		.head_block = &hda_log_dma_blk_cfg,
		.source_data_size = 4,
	};

	res = dma_config(hda_log_dev, hda_log_chan, &hda_log_dma_cfg);
	__ASSERT(res == 0, "DMA config failed");

	res = dma_start(hda_log_dev, hda_log_chan);
	__ASSERT(res == 0, "DMA start failed");

	atomic_set_bit(&hda_log_flags, HDA_LOG_DMA_READY);

	k_timer_init(&hda_log_timer, hda_log_periodic, NULL);
	k_timer_start(&hda_log_timer,
		      K_MSEC(CONFIG_LOG_BACKEND_ADSP_HDA_FLUSH_TIME),
		      K_MSEC(CONFIG_LOG_BACKEND_ADSP_HDA_FLUSH_TIME));

}

#ifdef CONFIG_LOG_BACKEND_ADSP_HDA_CAVSTOOL

#include <intel_adsp_ipc.h>
#include <cavstool.h>

#define CHANNEL 6
#define IPC_TIMEOUT K_MSEC(1500)

static inline void hda_ipc_msg(const struct device *dev, uint32_t data,
			       uint32_t ext, k_timeout_t timeout)
{
	__ASSERT(intel_adsp_ipc_send_message_sync(dev, data, ext, timeout),
		"Unexpected ipc send message failure, try increasing IPC_TIMEOUT");
}

/* Each try multiplies the delay by 2, 2^4*4000 is 64000 or 64ms
 * at the longest try period. Total try period is
 * 64000 + 32000 + 16000 + 8000 + 4000 or... 124 ms  */
#define HDA_NOTIFY_MAX_TRIES 6
#define DELAY_INIT 4000

void adsp_hda_log_cavstool_hook2(uint32_t hook_notify)
{
	hda_ipc_msg(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_HDA_PRINT, (hook_notify << 8) | CHANNEL, K_FOREVER);
}

void adsp_hda_log_cavstool_hook(uint32_t hook_notify)
{
	uint32_t try_loop = 0;
	bool done = false;
	uint32_t delay = DELAY_INIT;

	//DBG("send ipc start");

	/*  Send IPC message notifying log data has been written */
	do {
		done = intel_adsp_ipc_send_message(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_HDA_PRINT,
					     (hook_notify << 8) | CHANNEL);
		if(!done) {
			k_busy_wait(delay);
			delay = delay*2;
		}
		try_loop++;
	} while (!done && try_loop < HDA_NOTIFY_MAX_TRIES);

	if(try_loop >= HDA_NOTIFY_MAX_TRIES - 1) {
		DBG("send ipc fail");
		return;
	} else {
		//DBG("send ipc done");
	}

	try_loop = 0;
	delay = DELAY_INIT;

	//DBG("poll ipc start");
	
	/* Wait for a reply from the host
	 *
	 * Occasionally it seems we just lose IPC returns from the host (cavstool)
	 * so after a few tries we can move on and assume it worked hoping for the best
	 */
	do {
		done = intel_adsp_ipc_is_complete(INTEL_ADSP_IPC_HOST_DEV);
		if (!done) {
			k_busy_wait(delay);
			delay = delay*2;
		}
		try_loop++;
	} while (!done && try_loop < HDA_NOTIFY_MAX_TRIES);

	if(try_loop >= HDA_NOTIFY_MAX_TRIES) {
		DBG("poll ipc fail");
	} else {
		//DBG("poll ipc done");
	}
}

int adsp_hda_log_cavstool_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	hda_ipc_msg(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_HDA_RESET, CHANNEL, IPC_TIMEOUT);
	hda_ipc_msg(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_HDA_CONFIG,
		    CHANNEL | ((CONFIG_LOG_BACKEND_ADSP_HDA_SIZE) << 8), IPC_TIMEOUT);
	adsp_hda_log_init(adsp_hda_log_cavstool_hook, CHANNEL);
	hda_ipc_msg(INTEL_ADSP_IPC_HOST_DEV, IPCCMD_HDA_START, CHANNEL, IPC_TIMEOUT);

	return 0;
}

SYS_INIT(adsp_hda_log_cavstool_init, POST_KERNEL, 99);

#endif /* CONFIG_LOG_BACKEND_ADSP_HDA_CAVSTOOL */
