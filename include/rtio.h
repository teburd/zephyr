/*
 * Copyright (c) 2019  Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_RTIO_H_
#define ZEPHYR_INCLUDE_RTIO_H_
/**
 * @brief RTIO Interface
 * @defgroup rtio_interface RTIO Interface
 * @ingroup io_interfaces
 * @{
 */

#include <kernel.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Real-Time IO device API for moving bytes around fast
 *
 * Each Real-Time IO device provides a way to either read, write, or both
 * blocks of memory allocated from a static pool by the application. The layout
 * of the each block is understood implicitly by the author. Each device may
 * provide functionality to safely encode or decode blocks of memory.
 */

/**
 * @brief Chunk of memory with ownership and byte format versioning
 */
struct rtio_block {
	/* Mem block from k_mem_pool */
	struct k_mem_block_id mem_block_id;

	/** Timestamp in cycles from k_cycle_get_32() marking when a read or
	 * write began
	 */
	u32_t begin_tstamp;

	/** Timestamp in cycles from k_cycle_get_32() marking when a read or
	 * write was complete
	 */
	u32_t end_tstamp;

	/** Byte layout designator given by each driver to specify
	 * the layout of the contained buffer
	 */
	u32_t layout;

	/** Length of data in this buffer. */
	u16_t len;

	/** Amount of data this buffer can store. */
	u16_t size;

	/** Start of the buffer data .*/
	u8_t *_data;
};

/**
 * @brief Initialize an rtio_block
 */
__syscall void rtio_block_init(struct rtio_block *block,
				   struct k_mem_block_id mem_block_id,
			       u8_t *data, u32_t size);

static inline void z_impl_rtio_block_init(struct rtio_block *block,
				   struct k_mem_block_id mem_block_id,
				   u8_t *data, u32_t size)
{
	block->mem_block_id = mem_block_id;
	block->layout = 0;
	block->begin_tstamp = 0;
	block->end_tstamp = 0;
	block->len = 0;
	block->size = size;
	block->_data = data;
}

/**
 * @brief Begin a block read or write
 */
__syscall void rtio_block_begin(struct rtio_block *block);

static inline void z_impl_rtio_block_begin(struct rtio_block *block)
{
	block->len = 0;
	block->begin_tstamp = k_cycle_get_32();
}

/**
 * @brief End a block read or write.
 */
__syscall void rtio_block_end(struct rtio_block *block);

static inline void z_impl_rtio_block_end(struct rtio_block *block)
{
	block->end_tstamp = k_cycle_get_32();
}


/**
 * @brief Unused number of bytes in the buffer
 */
__syscall u16_t rtio_block_available(struct rtio_block *block);

static inline u16_t z_impl_rtio_block_available(struct rtio_block *block)
{
	return block->size - block->len;
}

/**
 * @private
 * @brief Implement a push/pull function pair
 *
 * Implements a pair of functions for pushing or pulling a fixed size value
 * into an rtio block.
 */
#define Z_RTIO_BLOCK_PUSH_PULL_IMPL(_type)				\
	static inline int rtio_block_push_##_type(			\
		struct rtio_block *block, _type val)			\
	{								\
		if (rtio_block_available(block) >= sizeof(_type)) {	\
			memcpy(&block->_data[block->len], &val,		\
			       sizeof(_type));				\
			block->len += sizeof(_type);			\
			return 0;				        \
		} else {						\
			return -ENOMEM;					\
		}							\
	}								\
									\
	static inline int rtio_block_pull_##_type(			\
		struct rtio_block *block, u16_t offset, _type * val)	\
	{								\
		if (block->len - offset >= sizeof(_type)) {		\
			memcpy(val, &block->_data[offset],		\
			       sizeof(_type));				\
			return offset + sizeof(_type);			\
		} else {						\
			return -EINVAL;					\
		}							\
	}

/**
 * @brief Push/pull a value in/out of the block
 *
 * Implementations for common data types you might wish to
 * store in a rtio_block
 */
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u8_t)
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u16_t)
Z_RTIO_BLOCK_PUSH_PULL_IMPL(u32_t)
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s8_t)
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s16_t)
Z_RTIO_BLOCK_PUSH_PULL_IMPL(s32_t)

/**
 * @brief An rtio pool
 */
struct rtio_pool {
	struct k_mem_pool *mempool;
};

/**
 * @brief Define an rtio block pool
 */
#define RTIO_POOL_DEFINE(_name, _maxsz, _nmax) \
	K_MEM_POOL_DEFINE(rtio_pool_mempool_##_name, 64, _maxsz, _nmax, 4); \
	struct rtio_pool _name = { \
		.mempool = &rtio_pool_mempool_##_name \
	}


/**
 * @brief Allocate a rtio_block from an rtio_pool with a timeout
 *
 * This call is not safe to do with a timeout other than K_NO_WAIT
 * in an interrupt handler.
 */
__syscall int rtio_block_alloc(struct rtio_pool *pool,
			       struct rtio_block **block,
			       u32_t size,
			       u32_t timeout);

static inline int
z_impl_rtio_block_alloc(struct rtio_pool *pool,
			struct rtio_block **block,
			u32_t size,
			u32_t timeout) {
	struct k_mem_block memblock;
	size_t block_size = size + sizeof(struct rtio_block);

	int res = k_mem_pool_alloc(pool->mempool, &memblock, block_size,
				   timeout);
	if (res == 0) {
		*block = (struct rtio_block *)(memblock.data);
		size_t struct_size = sizeof(struct rtio_block);
		u8_t *dataptr = &((u8_t *)memblock.data)[struct_size];

		rtio_block_init(*block, memblock.id, dataptr, size);
	}
	return res;
}

/**
 * @brief Free a rtio_block back to the pipe
 */
__syscall void rtio_block_free(struct rtio_block *block);

static inline void
z_impl_rtio_block_free(struct rtio_block *block) {
	k_mem_pool_free_id(&block->mem_block_id);
}
/**
 * @brief An rtio pipe provides a FIFO of rtio events (blocks, triggers, etc)
 */
struct rtio_pipe {
	struct k_fifo *queue;
};

/**
 * @brief Initialize a rtio pipe
 */
__syscall void rtio_pipe_init(struct rtio_pipe *pipe);

static inline void z_impl_rtio_pipe_init(struct rtio_pipe *pipe)
{
	k_fifo_init(pipe->queue);
}

/**
 * @brief Pull a block from the pipe with a timeout
 *
 * This call pulls a rtio_block from the rtio_pipe with a timeout.
 * If K_NO_WAIT is given results are immediate.
 * If K_FOREVER is given this call will wait until a rtio_block is ready.
 */
__syscall struct rtio_block *rtio_pipe_pull_block(
	struct rtio_pipe *pipe, u32_t timeout);

static inline struct rtio_block *
z_impl_rtio_pipe_pull_block(struct rtio_pipe *pipe, u32_t timeout) {
	return k_fifo_get(pipe->queue, timeout);
}

/**
 * @brief Push a block into the pipe
 *
 * This call pushes a rtio_block from the rtio_pipe making itavailable to be
 * pulled.
 */
__syscall void rtio_pipe_push_block(struct rtio_pipe *pipe,
				    struct rtio_block *block);

static inline void
z_impl_rtio_pipe_push_block(struct rtio_pipe *pipe, struct rtio_block *block) {
	return k_fifo_put(pipe->queue, block);
}


/**
 * @brief Define an rtio pipe
 */
#define RTIO_PIPE_DEFINE(_name)			\
	K_FIFO_DEFINE(rtio_blocks_queue_##_name);	\
	static struct rtio_pipe _name = {		\
		.queue = &rtio_blocks_queue_##_name	\
	}

/**
 * @brief Output config describing where and when to push a blocks
 *
 * Output config should at least have a valid pool and pipe pointer given
 *
 * Drivers should make a best attempt at fulfilling the policy of when
 * to return and notify the caller the block is ready by pushing it
 * into the rtio_pipe.
 *
 * If the buffer is full before the time or the desired number of
 * items is fulfilled its likely an error on the authors part and should
 * be logged as a warning when possible. This is a buffer overrun
 * scenario on the reader side.
 *
 * The behavior when the time is K_FOREVER and number is 0 the buffer
 * will be marked completed immediately and no read or write will take place.
 */
struct rtio_output_config {
	/**
	 * Pool to allocate blocks from
	 */
	struct rtio_pool *pool;

	/**
	 * Pipe to output to
	 */
	struct rtio_pipe *pipe;

	/** Time in cycles to read before making the block ready
	 *
	 * Should be set using something like K_MSEC(5) or K_FOREVER
	 */
	u32_t time;

	/** The number of bytes to read before making the block ready
	 * Note that the byte size should be computed by the driver from
	 * the size of the sample set multiplied by some number of samples
	 */
	u16_t byte_size;
};

/**
 * @brief Common trigger sources
 */
enum rtio_trigger_source {
	GPIO_IRQ,
	COUNTER_IRQ,
	FUNCTION_CALL,

	/* All driver specific sources should use this plus an additive value
	 * to label the trigger source
	 */
	RTIO_TRIGGER_SOURCE_COUNT
};

/**
 * @brief Trigger source configuration
 *
 * The source of the trigger may be an internal event along with the
 * associated GPIO device and pin, a counter timeout, or a function call
 */
struct rtio_trigger_config {
	enum rtio_trigger_source source;
	union {
		struct rtio_trigger_gpio {
			struct device *gpio;
			u32_t pin;
		} gpio_config;
		struct rtio_trigger_counter {
			struct device *counter;
			u32_t timeout;
		} counter_config;
	} options;
};

/**
 * @brief Trigger event used in rtio_pipe.
 */
struct rtio_trigger_event {
	u32_t trigger_source;
	u32_t timestamp;
};

/**
 * @brief Check if a block has met an output policy expectations
 *
 * @return true if the policy has been met, false otherwise
 */
__syscall bool rtio_output_policy_check(struct rtio_output_config *cfg,
					struct rtio_block *block);

static inline bool
z_impl_rtio_output_policy_check(struct rtio_output_config *cfg,
				struct rtio_block *block)
{
	if (k_cycle_get_32() - block->begin_tstamp > cfg->time) {
		return true;
	}
	if (block->len > cfg->byte_size) {
		return true;
	}
	return false;
}

/**
 * @brief A rtio configuration
 */
struct rtio_config {
	/* output configuration if applicable */
	struct rtio_output_config *output_config;

	/* trigger configuration if applicable */
	struct rtio_trigger_config *trigger_config;

	/* driver specific configuration */
	void *driver_config;
};

/**
 * @private
 * @brief Function definition for configuring a RTIO device
 */
typedef int (*rtio_configure_t)(struct device *dev,
				  struct rtio_config *config,
				  u32_t *version);

/**
 * @private
 * @brief Function definition for triggering a device read or write
 */
typedef int (*rtio_trigger_t)(struct device *dev);

/**
 * @brief Real-Time IO API
 */
struct rtio_api {
	/* Configuration function pointer *must* be implemented */
	rtio_configure_t configure;

	/* Trigger function pointer *must* be implemented and IRQ safe */
	rtio_trigger_t trigger;
};

/**
 * @brief Configure the device
 *
 * This will reconfigure the device given a device specific configuration
 * structure and possibly updates the block layout. This call will wait for
 * any pending IO operations to be completed and prevent further IO operations
 * until complete.
 *
 * Layout is assigned to the new buffer byte layout for the rtio device.
 * Its expected the caller understands the new byte layout.
 *
 * @param dev RTIO device to configure
 * @param config Configuration settings
 * @param[out] layout New block layout id after reconfiguring
 */
__syscall int rtio_configure(struct device *dev,
			     struct rtio_config *config,
			     u32_t *layout);

static inline int z_impl_rtio_configure(struct device *dev,
					struct rtio_config *config,
					u32_t *layout)
{
	const struct rtio_api *api = dev->driver_api;

	if (!api->configure) {
		return -ENOTSUP;
	}
	return api->configure(dev, config, layout);
}

/**
 * @brief Trigger a device read or write
 *
 * Triggers a read or write to be done by the device.
 *
 * @param dev RTIO device to trigger
 */
__syscall int rtio_trigger(struct device *dev);

static inline int z_impl_rtio_trigger(struct device *dev)
{
	const struct rtio_api *api = dev->driver_api;

	if (!api->trigger) {
		return -ENOTSUP;
	}
	return api->trigger(dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/rtio.h>

#endif
