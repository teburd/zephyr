/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_CAVS_HDA_H
#define ZEPHYR_INCLUDE_CAVS_HDA_H

#include "arch/xtensa/cache.h"
#include <kernel.h>
#include <device.h>
#include <cavs-shim.h>

/* HDA stream */
struct cavs_hda_stream {
	/* rather than copying data to and from
	 * the underlying fifo buffer we allow software
	 * to borrow slices of the buffer as a pointer
	 * and return it when done.
	 */
	uint32_t borrowed;

	/* track the hardware read pointer position in software
	 * to detect hardware updates
	 */
	uint32_t rp;

	/* track the hardware write pointer position
	 * to detect hardware updates
	 */
	uint32_t wp;

	/* track firmware pointer increment to
	 * ensure we don't overwrite existing buffer
	 * data when the hardware hasn't yet updated
	 * its own position.
	 */
	uint32_t fpi;

	/**
	 * buf
	 **/
	uint8_t *buf;
};

#define HDA_STREAM_COUNT 7

/* HDA stream set */
struct cavs_hda_streams {
	uint32_t base; /* base address of register block */
	struct cavs_hda_stream streams[HDA_STREAM_COUNT];
};

/**
 * Statically allocated and publicly available set of HDA stream sets
 */
struct cavs_hda {
	struct cavs_hda_streams host_in;
	struct cavs_hda_streams host_out;
};

static struct cavs_hda cavs_hda = {
	.host_out = {
		.base = 0x72800
	},
	.host_in = {
		.base = 0x72c00
	},
};

/* HDA IP Register Block */
#define HDA_REGBLOCK_SIZE 0x40
#define HDA_ADDR(base, stream) (base + stream*HDA_REGBLOCK_SIZE)

/* Buffer Size Minimum (32 bytes!) */
#define HDA_BUFSIZE_MIN 0x10

/* Read/Write pointer mask, 24 bits */
#define HDA_RWP_MASK 0x00FFFFFF

/* Gateway Control and Status Register */
#define DGCS(base, stream) ((volatile uint32_t *)HDA_ADDR(base, stream))
#define DGCS_SCS BIT(31) /* Sample container size */
#define DGCS_GEN BIT(26) /* Gateway Enable */
#define DGCS_L1ETP BIT(25) /* L1 Enter Prevent */
#define DGCS_L1EXP BIT(25) /* L1 Exit Prevent */
#define DGCS_FWCB BIT(23) /* Firmware Control Buffer */
#define DGCS_GBUSY BIT(15) /* Gateway Busy */
#define DGCS_TE BIT(14) /* Transfer Error */
#define DGCS_BSC BIT(11) /* Buffer Segment Completion */
#define DGCS_BOR BIT(10) /* Buffer Overrun */
#define DGCS_BUR BIT(10) /* Buffer Underrun */
#define DGCS_BF BIT(9) /* Buffer Full */
#define DGCS_BNE BIT(8) /* Buffer Not Empty */
#define DGCS_FIFORDY BIT(5) /* Enable FIFO */
#define DGCS_BSCIE BIT(3) /* Buffer Segment Completion Interrupt Enable */

/* Gateway Buffer Base Address */
#define DGBBA(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x04))
/* Gateway Buffer Size */
#define DGBS(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x08))
/* Gateway Buffer Pointer Increment */
#define DGBFPI(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x0c))
/* Gateway Buffer Read Pointer */
#define DGBRP(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x10))
/* Gateway Buffer Write Pointer */
#define DGBWP(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x14))
/* Gateway Buffer Segment Pointer */
#define DGBSP(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x18))
/* Gateway Minimum Buffer Size */
#define DGMBS(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x1c))
/* Gateway Linear Link Position Increment */
#define DGLLPI(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x24))
/* Gateway Linear Position In Buffer Increment */
#define DGLPIBI(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x28))


/**
 * @brief Dump all the useful registers of an HDA stream to printk
 *
 * This can be invaluable when finding out why HDA isn't doing (or maybe is)
 * doing what you want it to do. Macro so you get the file and line
 * of the call site included.
 */
#define cavs_hda_dbg(hda, sid)						\
	do {								\
		const char *hda_name;					\
		if (&cavs_hda.host_in == hda) {				\
			hda_name = "in";				\
		} else {						\
			hda_name = "out";				\
		}							\
		printk("%s:%u %s(%u:0x%p), dgcs: 0x%x, dgbba 0x%x, dgbs %u, dgbrp %u, dgbwp %u, dgbsp %u, dgmbs %u, dgbllpi 0x%x, dglpibi 0x%x\n", \
		       __FILE__, __LINE__, hda_name,			\
		       sid, DGCS(hda->base, sid),			\
		       *DGCS(hda->base, sid),				\
		       *DGBBA(hda->base, sid),				\
		       *DGBS(hda->base, sid),				\
		       *DGBRP(hda->base, sid),				\
		       *DGBWP(hda->base, sid),				\
		       *DGBSP(hda->base, sid),				\
		       *DGMBS(hda->base, sid),				\
		       *DGLLPI(hda->base, sid),				\
		       *DGLPIBI(hda->base, sid));			\
		printk("\tborrowed %d, wp: %d, rp: %d, fpi %d\n",	\
		       hda->streams[sid].borrowed,			\
		       hda->streams[sid].wp,				\
		       hda->streams[sid].rp,				\
		       hda->streams[sid].fpi);				\
	} while (0)


/**
 * @brief Set the buffer, size, and element size for an HDA stream
 *
 * Prior to enabling an HDA stream to/from the host this is the minimum configuration
 * that is required.
 *
 * @param hda Stream set to work with
 * @param sid Stream identifier
 * @param buf Buffer address to use for the shared FIFO. Must be in L2.
 * @param buf_size Buffer size in bytes
 * @param elem_size Buffer element size, must be *atleast* 0x10 bytes.
 * @retval -EBUSY if the HDA stream is already enabled
 * @retval -EINVAL if the buf is not in L2, buf isn't aligned on 128 byte boundaries
 * @retval 0 on Success
 */
static inline void cavs_hda_set_buffer(struct cavs_hda_streams *hda, uint32_t sid,
				     uint8_t *buf, uint32_t buf_size)
{

	/* Tell hardware we want to control the buffer pointer,
	 * we don't want it to enter low power, and we want to control
	 * the sample size.
	 */
	/* Sample size is 4 bytes when scs not set */
	*DGCS(hda->base, sid) |= DGCS_FWCB | DGCS_L1ETP;
	void *cached_buf = arch_xtensa_cached_ptr(buf);
	uint32_t cached_addr = (uint32_t)cached_buf;
	uint32_t aligned_addr = cached_addr & 0x0FFFFF80;


	*DGBBA(hda->base, sid) = aligned_addr;

	*DGBS(hda->base, sid) = buf_size;
	*DGMBS(hda->base, sid) = 256;

	hda->streams[sid].borrowed = 0;
	hda->streams[sid].rp = 0;
	hda->streams[sid].wp = 0;
	hda->streams[sid].fpi = 0;
	hda->streams[sid].buf = buf;
}

/**
 * @brief Force DMA to exit L1 (low power state)
 */
static inline void cavs_hda_l1_exit(struct cavs_hda_streams *hda, uint32_t sid)
{
	CAVS_SHIM.svcfg |= BIT(2);
	k_msleep(1);
	CAVS_SHIM.svcfg &= ~BIT(2);
}

/**
 * @brief Enable the stream
 *
 * @param hda HDA stream to enable
 * @param sid Stream identifier
 */
static inline void cavs_hda_enable(struct cavs_hda_streams *hda, uint32_t sid)
{
	/* enable the stream */
	*DGCS(hda->base, sid) |= DGCS_GEN | DGCS_FIFORDY;
}

/**
 * @brief Disable stream
 *
 * @param hda HDA stream to disable */
static inline void cavs_hda_disable(struct cavs_hda_streams *hda, uint32_t sid)
{
	/* disable the stream */
	/* TODO maybe see if we can signal a stop with FIFORDY
	 * by setting high, waiting for the appropriate index
	 * to stop incrementing or GBUSY bit, then unsetting DGCS_GEN
	 * once the stream is stopped
	 */
	*DGCS(hda->base, sid) &= ~(DGCS_GEN | DGCS_FIFORDY);
}

/**
 *@brief Determine the number of unused bytes in the fifo
 *
 * @param hda HDA device
 * @param sid Stream ID
 *
 * @retval n Number of unused bytes
 */
static inline uint32_t cavs_hda_unused(struct cavs_hda_streams *hda, uint32_t sid)
{
	uint32_t dgcs = *DGCS(hda->base, sid);

	/* if buffer is full then 0 bytes available
	 * TODO this only works if the host sets the stream format!
	 * which means its not useful in a general fifo sense but
	 * in I think a flow control sense. "don't write more, backlogged" kind of
	 * thing.
	 */
	/*
	if (dgcs & DGCS_BF) {
		printk("buffer full!\n");
		return 0;
	}
	*/

	uint32_t dgbs = *DGBS(hda->base, sid);

	/* if buffer is not empty */
	if ((dgcs & DGCS_BNE) == 0) {
		printk("buffer empty!\n");
		return dgbs;
	}

	int32_t rp = *DGBRP(hda->base, sid);
	int32_t wp = *DGBWP(hda->base, sid);
	int32_t size = rp - wp;

	if (size <= 0) {
		size += dgbs;
	}

	printk("cavs_hda_unused stream %d, size %d\n", sid, size);
	return size;
}

/**
 * @brief Update the firmware position by a given length
 *
 * Updates the ringbuffer position (read or write pointer) by
 * setting the FPI register. We update the book kept state of
 * the stream rp/wp prior to the update of position. In
 * some cases where the hardware is not pulling yet from
 * the FIFO the WP/RP will not update until it is started.
 *
 * @param hda HDA device
 * @param sid Stream ID
 * @param len Len to increment postion by
 */
static inline void cavs_hda_inc_pos(struct cavs_hda_streams *hda, uint32_t sid, uint32_t len)
{
	*DGBFPI(hda->base, sid) = len;
	*DGLLPI(hda->base, sid) = len;
	*DGLPIBI(hda->base, sid) = len;
}

/**
 * @brief Copying write to the underlying HDA FIFO Buffer.
 *
 * Copies bytes from buf to the underlying stream fifo wrapping if needed
 * automatically.
 *
 * @param hda HDA device
 * @param sid Stream ID
 * @param buf Buffer of bytes
 * @param buf_len Number of bytes
 *
 * @retval -EAGAIN Previous writes to the buffer have not been committed in hardware
 *            or the stream isn't enabled in some way. Check on the host SDxCTL.RUN and
 *            ppctl.PPROC are set. Check on the DSP that dgctl.GEN is enabled.
 * @retval -ENOBUFS Not enough room in the FIFO for the buffer to be written. Try again soon
 *            and ensure the stream is running and enabled.
 * @retval  rem Write was successful and the remaining number of bytes to be read or
 *            writen by the hardware is returned.
 */
static inline int cavs_hda_write(struct cavs_hda_streams *hda, uint32_t sid,
				 uint8_t *buf, uint32_t buf_len)
{
	/* TODO assert on buf_len being word aligned? would avoid the byte-wise copy below */
	cavs_hda_dbg(hda, sid);

	uint32_t dgbwp = *DGBWP(hda->base, sid);

	/* If the hardware hasn't updated the write pointer and we've written previously
	 * there's an issue and we cannot continue.
	 */
	if (dgbwp == hda->streams[sid].wp && hda->streams[sid].fpi > 0) {
		return -EAGAIN;
	}

	/* Check if there's room in the fifo for the given buffer */
	if (cavs_hda_unused(hda, sid) < buf_len) {
		return -ENOBUFS;
	}

	/* Start on the current write position
	 * then increment by one before each write
	 *
	 * TODO use memcpy rather than byte copies when feasible, split when wrapped
	 */
	uint32_t fifo_size = *DGBS(hda->base, sid);
	uint8_t *fifo = arch_xtensa_uncached_ptr(hda->streams[sid].buf);
	uint32_t idx = dgbwp;
	for(uint32_t i = 0; i < buf_len; i++) {
		idx++;
		/* wrapped index case */
		if (idx == fifo_size) {
			idx = 0;
		}
		fifo[idx] = buf[i];
	}

	/* Indicate we've provided buf_len bytes and flag the buffer segment complete bit */
	*DGCS(hda->base, sid) &= ~DGCS_BSC; /* clear the bsc bit if set by the hardware */
	*DGBSP(hda->base, sid) = idx; /* have the hardware set the BSC bit again when we've reached the end of our last write */

	cavs_hda_inc_pos(hda, sid, buf_len);

	/* book keeping to ensure we don't corrupt our own buffer by
	 * writing again  while the hardware is disabled or hasn't yet picked up
	 * our position change which while unlikely seems to be possible.
	 */
	hda->streams[sid].wp = dgbwp;
	hda->streams[sid].rp = *DGBRP(hda->base, sid);
	hda->streams[sid].fpi = buf_len;

	return (buf_len - cavs_hda_unused(hda, sid));
}

static inline bool cavs_hda_wp_rp_eq(struct cavs_hda_streams *hda, uint32_t sid)
{
	return *DGBWP(hda->base, sid) == *DGBRP(hda->base, sid);
}

#endif // ZEPHYR_INCLUDE_CAVS_HDA_H_
