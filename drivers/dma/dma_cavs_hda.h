/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DMA_DMA_CAVS_HDA_H_
#define ZEPHYR_DRIVERS_DMA_DMA_CAVS_HDA_H_


/* HDA IP Register Block */
#define HDA_REGBLOCK_SIZE 0x40
#define HDA_ADDR(base, stream) (base + stream*HDA_REGBLOCK_SIZE)

/* Gateway Control and Status Register */
#define DGCS(base, stream) ((volatile uint32_t *)HDA_ADDR(base, stream))
#define DGCS_SCS BIT(31) /* Sample container size */
#define DGCS_GEN BIT(26) /* Gateway Enable */
#define DGCS_L1ETP BIT(25) /* L1 Enter Prevent */
#define DGCS_L1EXP BIT(25) /* L1 Exit Prevent */
#define DGCS_FWCB BIT(23) /* Firmware Control Buffer */
#define DGCS_GBUSY BIT(15) /* Gateway Busy */
#define DGCS_TE BIT(14) /* Transfer Error */
#define DGCS_BSC BIT(11) /* Buffer segment completion */
#define DGCS_BOR BIT(10) /* Buffer Overrun */
#define DGCS_BUR BIT(10) /* Buffer Underrun */
#define DGCS_BF BIT(9) /* Buffer Full */
#define DGCS_BNE BIT(8) /* Buffer not empty */
#define DGCS_FIFORDY BIT(5) /* Enable FIFO */

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


/* Host only registers that don't apply to link */

/* Gateway Minimum Buffer Size */
#define DGMBS(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x1c))

/* Gateway Linear Link Position Increment */
#define DGLLPI(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x24)

/* Gateway Linear Position In Buffer Increment */
#define DGLPIBI(base, stream) ((volatile uint32_t *)(HDA_ADDR(base, stream) + 0x28)



#endif /* ZEPHYR_DRIVERS_DMA_DMA_CAVS_HDA_H_ */
