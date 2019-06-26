/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_RTIO_FIFO_H_
#define ZEPHYR_RTIO_FIFO_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/atomic.h>

/**
 * @brief RTIO FIFO API
 * @defgroup rtio_fifo RTIO FIFO API
 * @ingroup rtio
 * @{
 */

/**
 * @file rtio_fifo.h
 *
 * @brief A lock-free and type safe power of 2 SPSC Queue (FIFO) using a
 * ringbuffer and atomics to ensure coherency.
 *
 * This FIFO implementation works on an array which wraps using a power of two
 * size and uses a bit mask to perform a modulus. Atomics are used to allow
 * single-producer single-consumer safe semantics without locks. Elements are
 * expected to be of a fixed size. The API is type safe as the underlying buffer
 * is typed and all usage is done through macros.
 *
 * A fifo may be declared on a stack or statically and work as intended so long
 * as its lifetime outlives any usage. Static declarations should be the
 * preferred method. It is meant to be a shared object between two execution
 * contexts (ISR and a thread for example)
 *
 * The FIFO is safe to produce or consume in an ISR with O(1) push/pull.
 *
 * The FIFO is *not* safe to produce in multiple execution contexts or consume in
 * multiple execution contexts without additional work.
 *
 * Safe usage would be, where A and B are unique execution contexts:
 * 1. ISR A producing and a Thread B consuming.
 * 2. Thread A producing and ISR B consuming.
 * 3. Thread A producing and Thread B consuming.
 */

/**
 * @private
 * @brief Common fifo attributes
 *
 * @warning Not to be manipulated without the macros!
 */
struct rtio_fifo {
	/* private value only the producer thread should mutate */
	uint32_t acquire;

	/* private value only the consumer thread should mutate */
	uint32_t consume;

	/* producer mutable, consumer readable */
	atomic_t in;

	/* consumer mutable, producer readable */
	atomic_t out;

	/* mask used to automatically wrap values */
	const uint32_t mask;
};

/**
 * @brief Statically initialize a rtio_fifo
 */
#define RTIO_FIFO_INITIALIZER(name, type, sz)	  \
	{ ._fifo = {				  \
		  .acquire = 0,			  \
		  .consume = 0,			  \
		  .in = ATOMIC_INIT(0),		  \
		  .out = ATOMIC_INIT(0),	  \
		  .mask = sz - 1,		  \
	  }					  \
	}

/**
 * @brief Declare an anonymous struct type for a rtio_fifo
 */
#define RTIO_FIFO_DECLARE(name, type, sz) \
	struct rtio_fifo_ ## name {	  \
		struct rtio_fifo _fifo;	  \
		type buffer[sz];	  \
	} _ ## name

/**
 * @brief Define a rtio_fifo with a fixed size
 *
 * @param name Name of the fifo
 * @param type Type stored in the fifo
 * @param sz Size of the fifo, must be power of 2 (ex: 2, 4, 8)
 */
#define RTIO_FIFO_DEFINE(name, type, sz)                                            \
	RTIO_FIFO_DECLARE(name, type, sz) = RTIO_FIFO_INITIALIZER(name, type, sz);  \
	struct rtio_fifo_##name * const name = &_##name

/**
 * @brief Size of the fifo
 *
 * @param fifo FIFO reference
 */
#define rtio_fifo_size(fifo) ((fifo)->_fifo.mask + 1)

/**
 * @private
 * @brief A number modulo the fifo size, assumes power of 2
 *
 * @param fifo FIFO reference
 * @param i Value to modulo to the size of the fifo
 */
#define z_rtio_fifo_mask(fifo, i) (i & (fifo)->_fifo.mask)

/**
 * @brief Initialize/reset a fifo such that its empty
 *
 * Note that this is not safe to do while being used in a producer/consumer
 * situation with multiple calling contexts (isrs/threads).
 *
 * @param fifo FIFO to initialize/reset
 */
#define rtio_fifo_reset(fifo)                                                                      \
	({                                                                                         \
		(fifo)->_fifo.consume = 0;                                                         \
		(fifo)->_fifo.acquire = 0;                                                         \
		atomic_set(&(fifo)->_fifo.in, 0);                                                  \
		atomic_set(&(fifo)->_fifo.out, 0);                                                 \
	})

/**
 * @brief Acquire an element to produce from the FIFO
 *
 * @param fifo FIFO to acquire an element from for producing
 *
 * @return A pointer to the acquired element or null if the fifo is full
 */
#define rtio_fifo_acquire(fifo)                                                                    \
	({                                                                                         \
		uint32_t idx = (uint32_t)atomic_get(&(fifo)->_fifo.in) + (fifo)->_fifo.acquire;    \
		bool acq =                                                                         \
			(idx - (uint32_t)atomic_get(&(fifo)->_fifo.out)) < rtio_fifo_size(fifo);   \
		if (acq) {                                                                         \
			(fifo)->_fifo.acquire += 1;                                                \
		}                                                                                  \
		acq ? &((fifo)->buffer[z_rtio_fifo_mask(fifo, idx)]) : NULL;                       \
	})

/**
 * @brief Produce one previously acquired element to the FIFO
 *
 * This makes one element available to the consumer immediately
 *
 * @param fifo FIFO to produce the previously acquired element or do nothing
 */
#define rtio_fifo_produce(fifo)					\
	({							\
		if ((fifo)->_fifo.acquire > 0) {		\
			(fifo)->_fifo.acquire -= 1;		\
			atomic_add(&(fifo)->_fifo.in, 1);	\
		}						\
	})

/**
 * @brief Consume a element from the fifo
 *
 * @param fifo Fifo to peek into
 *
 * @return Pointer to element or null if no consumable elements left
 */
#define rtio_fifo_consume(fifo)						\
	({								\
		uint32_t idx = (uint32_t)atomic_get(&(fifo)->_fifo.out) + (fifo)->_fifo.consume; \
		bool has_consumable =  (idx != (uint32_t)atomic_get(&(fifo)->_fifo.in)); \
		if (has_consumable) {					\
			(fifo)->_fifo.consume += 1;			\
		}							\
		has_consumable ? &((fifo)->buffer[z_rtio_fifo_mask(fifo, idx)]) : NULL; \
	})

/**
 * @brief Release a consumed element
 *
 * @param fifo FIFO to release consumed element or do nothing
 */
#define rtio_fifo_release(fifo)					\
	({							\
		if ((fifo)->_fifo.consume > 0) {		\
			(fifo)->_fifo.consume -= 1;		\
			atomic_add(&(fifo)->_fifo.out, 1);	\
		}						\
	})

/**
 * @brief Count of consumables in fifo
 *
 * @param fifo FIFO to get item count for
 */
#define rtio_fifo_consumable(fifo)			\
	({						\
		(fifo)->_fifo.in \
			- (fifo)->_fifo.out \
			- (fifo)->_fifo.consume;	\
	})						\

/**
 * @}
 */

#endif /* ZEPHYR_RTIO_FIFO_H_ */
