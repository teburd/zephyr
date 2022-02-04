/* Copyright (c) 2022 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_CAVS_HDA_H
#define ZEPHYR_INCLUDE_CAVS_HDA_H

#include <kernel.h>
#include <device.h>
#include <cavs-shim.h>

/* HDA stream */
struct cavs_hda_stream {
	uint32_t elem_size;
	uint32_t borrowed;
	uint32_t last_brwp; /* track the hardware pointer position when last changed */
	uint32_t fpi; /* number of positions from the hardware pointer position expected to change */
};

#define HDA_STREAM_COUNT 14

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
} cavs_hda = {
	.host_in = {
		.base = 0x72c00
	},
	.host_out = {
		.base = 0x72800
	},
};

/* HDA IP Register Block */
#define HDA_REGBLOCK_SIZE 0x40
#define HDA_ADDR(base, stream) (base + stream*HDA_REGBLOCK_SIZE)

/* Buffer Size Minimum */
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
 */
#define cavs_hda_dbg(hda, sid)						\
	do {								\
		const char *hda_name;					\
		if (&cavs_hda.host_in == hda) {				\
			hda_name = "in";				\
		} else {						\
			hda_name = "out";				\
		}							\
		printk("%s:%u %s(%u:0x%p), dgcs: 0x%x, dgbba 0x%x, dgbs %u, dgbrp 0x%x, dgbwp 0x%x, dgbsp 0x%x, dgmbs %u, dgbllpi 0x%x, dglpibi 0x%x\n", \
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
		printk("\tlast_brwp %d, fpi %d\n", hda->streams[sid].last_brwp, hda->streams[sid].fpi); \
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
 * @retval -EINVAL if the buf is not in L2, buf size isn't aligned on 0x10, or elem_size is smaller than 0x10
 * @retval 0 on Success
 */
static inline void cavs_hda_set_buffer(struct cavs_hda_streams *hda, uint32_t sid,
				     uint8_t *buf, uint32_t buf_size, uint16_t elem_size)
{
	cavs_hda_dbg(hda, sid);

	/* Tell hardware we want to control the buffer pointer
	 * and we haven't read anything yet so don't power down
	 */
	*DGCS(hda->base, sid) |= DGCS_FWCB | DGCS_L1ETP;

	*DGBBA(hda->base, sid) = (uint32_t)buf;
	*DGBS(hda->base, sid) = HDA_RWP_MASK & buf_size;
	*DGMBS(hda->base, sid) = elem_size;

	cavs_hda_dbg(hda, sid);
	hda->streams[sid].borrowed = 0;
	hda->streams[sid].last_brwp = 0;
	hda->streams[sid].fpi = 0;

}

/**
 * @brief Force DMA to exist L1 (low power state)
 */
static inline void cavs_hda_l1_exit(struct cavs_hda_streams *hda, uint32_t sid)
{
	CAVS_SHIM.svcfg |= BIT(2);
	k_busy_wait(100000);
	CAVS_SHIM.svcfg &= ~BIT(2);
}


/**
 * @brief Enable the stream
 *
 * @param hda Stream set to work with
 * @param sid Stream identifier
 */
static inline void cavs_hda_enable(struct cavs_hda_streams *hda, uint32_t sid)
{
	/* enable the stream */
	*DGCS(hda->base, sid) |= DGCS_GEN;
}

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
 * Write a byte only waiting for a spot to write it if needed
 */
static inline int cavs_hda_write(struct cavs_hda_streams *hda, uint32_t sid, uint8_t byte)
{
	cavs_hda_dbg(hda, sid);

	uint32_t dgbwp = *DGBWP(hda->base, sid);

	/* If the hardware hasn't update the write pointer and we've written previously
	 * there's an issue
	 */
	printk("Check if writes have carried on in hardware...\n");
	if (dgbwp == hda->streams[sid].last_brwp && hda->streams[sid].fpi > 0) {
		return -1;
	}

	uint32_t next_idx = dgbwp + 1;

	/* wrapped next index */
	if (next_idx >= *DGBS(hda->base, sid)) {
		next_idx = 0;
	}

	cavs_hda_dbg(hda, sid);

	printk("Check if buffer is full, next_idx %d...\n", next_idx);
	uint32_t retries = 10;
	while (next_idx == *DGBRP(hda->base, sid)) {
		retries--;
		if (retries == 0) {
			return -2;
		}
	}

	printk("Writing byte(s)\n");
	((volatile uint32_t *)(*DGBBA(hda->base, sid)))[next_idx] = byte;

	printk("Indicate byte(s) written\n");
	/* Indicate we've provided 1 byte */
	*DGBFPI(hda->base, sid) += 1;
	cavs_hda_dbg(hda, sid);

	/* book keeping to ensure we don't corrupt our own buffer by
	 * writing faster than the hardware
	 */
	hda->streams[sid].last_brwp = dgbwp;
	hda->streams[sid].fpi = 1;

	cavs_hda_l1_exit(hda, sid);
	return 1;
}

/*
static inline int cavs_hda_unused(struct cavs_hda_streams *hda, uint32_t sid)
{
	uint32_t dgcs = *DGCS(hda->base, sid);

	if (dgcs & DGCS_BF) {
		return 0;
	}

	uint32_t dgbs = *DGBS(hda->base, sid);

	if (dgcs & DGCS_BNE == 0) {
		return dgbs;
	}

	int32_t rp = *DGBRP(hda->base, sid);
	int32_t wp = *DGBWP(hda->base, sid);
	int32_t size = rp - wp;

	if (size <= 0) {
		size +=dgbs;
	}

	return size;
}
*/




#endif // ZEPHYR_INCLUDE_CAVS_HDA_H_
