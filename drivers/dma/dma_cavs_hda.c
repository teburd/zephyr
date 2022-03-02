/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/dma.h>
#include <cavs_hda.h>
#include "dma_cavs_hda.h"

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(dma_cavs_hda_dma);

/**
 * @brief Intel CAVS HDA DMA (Stream) driver
 *
 * HDA is effectively, from the DSP, a ringbuffer (fifo) where the read
 * and write positions are maintained by the hardware and the software may
 * commit read/writes by writing to another register (DGFPBI) the length of
 * the read or write.
 *
 * It's important that the software knows the position in the ringbuffer to read
 * or write from. It's also important that the buffer be placed in the correct
 * memory region and aligned to 128 bytes. Lastly it's important the host and
 * dsp coordinate the order in which operations takes place. Doing all that
 * HDA streams are a fantastic bit of hardware and do their job well.
 *
 * There are 4 types of streams, with a set of each available to be used to
 * communicate to or from the Host or Link. Each stream set is uni directional.
 */


int cavs_hda_dma_host_in_config(const struct device *dev,
				       uint32_t channel,
				       struct dma_config *dma_cfg)
{
	const struct cavs_hda_dma_cfg *const cfg = dev->config;
	struct dma_block_config *blk_cfg;
	uint8_t *buf;

	__ASSERT(channel < cfg->dma_channels, "Channel does not exist");
	__ASSERT(dma_cfg->block_count == 1,
		 "HDA only support one short or circular style transfers, only "
		 "one block config allowed");

	blk_cfg = dma_cfg->head_block;
	buf = (uint8_t *)(uintptr_t)(blk_cfg->source_address);
	return cavs_hda_set_buffer(cfg->base, channel, buf,
				  blk_cfg->block_size);
}


int cavs_hda_dma_host_out_config(const struct device *dev,
					uint32_t channel,
					struct dma_config *dma_cfg)
{
	const struct cavs_hda_dma_cfg *const cfg = dev->config;
	uint8_t *buf;
	struct dma_block_config *blk_cfg;

	__ASSERT(channel < cfg->dma_channels, "Channel does not exist");
	__ASSERT(dma_cfg->block_count == 1,
		 "HDA only support one short or circular style transfers, only "
		 "one block config allowed");

	blk_cfg = dma_cfg->head_block;
	buf = (uint8_t *)(uintptr_t)(blk_cfg->dest_address);

	return cavs_hda_set_buffer(cfg->base, channel, buf,
				  blk_cfg->block_size);
}

int cavs_hda_dma_host_reload(const struct device *dev, uint32_t channel,
				    uint32_t src, uint32_t dst, size_t size)
{
	const struct cavs_hda_dma_cfg *const cfg = dev->config;

	__ASSERT(channel < cfg->dma_channels, "Channel does not exist");

	cavs_hda_commit(cfg->base, channel, size);

	return 0;
}

int cavs_hda_dma_status(const struct device *dev, uint32_t channel,
	struct dma_status *stat)
{
	const struct cavs_hda_dma_cfg *const cfg = dev->config;

	__ASSERT(channel < cfg->dma_channels, "Channel does not exist");

	stat->busy = *DGCS(cfg->base, channel) & DGCS_GBUSY;
	stat->write_position = *DGBWP(cfg->base, channel);
	stat->read_position = *DGBRP(cfg->base, channel);

	return 0;
}

int cavs_hda_dma_start(const struct device *dev, uint32_t channel)
{
	const struct cavs_hda_dma_cfg *const cfg = dev->config;

	__ASSERT(channel < cfg->dma_channels, "Channel does not exist");

	cavs_hda_enable(cfg->base, channel);

	return 0;
}

int cavs_hda_dma_stop(const struct device *dev, uint32_t channel)
{
	const struct cavs_hda_dma_cfg *const cfg = dev->config;

	__ASSERT(channel < cfg->dma_channels, "Channel does not exist");

	cavs_hda_disable(cfg->base, channel);

	return 0;
}

int cavs_hda_dma_init(const struct device *dev)
{
	struct cavs_hda_dma_data *data = dev->data;
	const struct cavs_hda_dma_cfg *const cfg = dev->config;

	for (uint32_t i = 0; i < cfg->dma_channels; i++) {
		cavs_hda_init(cfg->base, i);
	}

	data->ctx.dma_channels = cfg->dma_channels;
	data->ctx.atomic = data->channels_atomic;
	data->ctx.magic = DMA_MAGIC;

	LOG_INF("Intel cAVS HDA %s initialized", dev->name);

	return 0;
}
