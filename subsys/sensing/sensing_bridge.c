/*
 * Copyright (c) 2022-2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_sensing_bridge

#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sensing/sensing.h>
#include <zephyr/sensing/sensing_sensor.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>

#LOG_MODULE_REGISTER(sensing_bridge, CONFIG_SENSING_LOG_LEVEL);

/*
 * A sensing bridge may act as if it were one or more sensors
 * in one the graph of pub/sub nodes
 */
struct sensing_channel_bridge {
	enum sensor_channel sensor_channel;
	struct sensing_sensor_ctx context;
};

/*
 * Bridge the single owner sensor to a pub/sub sensing
 * DAG
 *
 * Each sensor may actually represent a number of streams of data
 * and each stream (channel) needs to be represented in the
 * subsystem DAG individually while a single sensor is generating
 * the data.
 */
struct sensing_bridge {
	const struct device *sensor;
	usize_t channels;
	struct sensing_channel_bridge channel_bridges[];
};


static int sensing_bridge_init(const struct device *dev,
		const struct sensing_sensor_info *info,
		const sensing_sensor_handle_t *reporter_handles,
		int reporters_count)
{
	return 0;
}

static int sensing_bridge_deinit(const struct device *dev)
{
	return 0;
}

static int sensing_bridge_read_sample(const struct device *dev,
		void *buf, int size)
{
	return 0;
}

static int sensing_bridge_sensitivity_test(const struct device *dev,
		int index, uint32_t sensitivity,
		void *last_sample_buf, int last_sample_size,
		void *current_sample_buf, int current_sample_size)
{
	return 0;
}

static int sensing_bridge_set_interval(const struct device *dev, uint32_t value)
{
	return 0;
}

static int sensing_bridge_get_interval(const struct device *dev,
		uint32_t *value)
{
	return 0;
}

static int sensing_bridge_set_sensitivity(const struct device *dev,
		int index, uint32_t value)
{
	return 0;
}

static int sensing_bridge_get_sensitivity(const struct device *dev,
		int index, uint32_t *value)
{
	return 0;
}

static const struct sensing_sensor_api sensing_bridge_api = {
	.init = sensing_bridge_init,
	.deinit = sensing_bridge_deinit,
	.set_interval = sensing_bridge_set_interval,
	.get_interval = sensing_bridge_get_interval,
	.set_sensitivity = sensing_bridge_set_sensitivity,
	.get_sensitivity = sensing_bridge_get_sensitivity,
	.read_sample = sensing_bridge_read_sample,
	.sensitivity_test = sensing_bridge_sensitivity_test,
};

static const struct sensing_sensor_register_info sensing_bridge_reg = {
	.flags = SENSING_SENSOR_FLAG_REPORT_ON_CHANGE,
	.sample_size = sizeof(struct sensing_sensor_value_3d_q31),
	.sensitivity_count = SENSING_BRIDGE_CHANNEL_NUM,
	.version.value = SENSING_SENSOR_VERSION(0, 8, 0, 0),
};

#define SENSING_SENSING_BRIDGE_DT_DEFINE(_inst)				\
	static struct sensing_bridge _CONCAT(ctx, _inst) = {	\
		.sensor = DEVICE_DT_GET(				\
				DT_PHANDLE(DT_DRV_INST(_inst),		\
				underlying_device)),			\
		.sensor_type = DT_PROP(DT_DRV_INST(_inst), sensor_type),\
	};								\
	SENSING_SENSOR_DT_DEFINE(DT_DRV_INST(_inst),			\
		&sensing_bridge_reg, &_CONCAT(ctx, _inst),		\
		&sensing_bridge_api);

DT_INST_FOREACH_STATUS_OKAY(SENSING_SENSING_BRIDGE_DT_DEFINE);
