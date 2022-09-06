/* 
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/__assert.h"
#define DT_DRV_COMPAT invensense_icm42688

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor_types.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/math/util.h>
#include "icm42688.h"
#include "icm42688_reg.h"
#include "icm42688_spi.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM42688, CONFIG_SENSOR_LOG_LEVEL);

struct icm42688_sensor_data {
	struct icm42688_dev_data dev_data;

	sensor_process_data_callback_t process_callback;
	struct sensor_raw_data *fifo_buf;
	
	struct gpio_callback gpio_cb1;
	struct k_sem int1_sem;
};

struct icm42688_sensor_config {
	struct icm42688_dev_cfg dev_cfg;
};

int16_t raw_to_cpu(uint8_t byte_h, uint8_t byte_l)
{
	return sys_le16_to_cpu((byte_h << 8) | byte_l);	
}

/** 
 * @NOTE Unclear what void *data actually is... is it a senseor_raw_data, sensor three axis?
 * The sensor type shouldn't probably define the data type.
 * 
 * @NOTE The fp_t math means I can't reuse the existing, working math, from the v1 API
 * without float unfortunately.
 *
 * @NOTE No guarantee the samples are of the same sampling clock instant either here
 */
static int icm42688_read_data(const struct device *dev, uint32_t sensor_type, void *sdata)
{
	int res;
	struct icm42688_sensor_data *sens_data = dev->data;
	struct icm42688_dev_data *ddata = &sens_data->dev_data;
	
	uint8_t data[14];
	
	res = icm42688_read_all(dev,  data);
	
	if (res != 0 ) {
		LOG_ERR("Error reading data from sensor");
		goto out;
	}
	
	
	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		struct sensor_three_axis_data *adata = sdata;
		int32_t ms[3];
		uint32_t ums[3];

		icm42688_accel_ms(&ddata->cfg, raw_to_cpu(data[2], data[3]),  &ms[0], &ums[0]);
		icm42688_accel_ms(&ddata->cfg, raw_to_cpu(data[4], data[5]),  &ms[1], &ums[1]);
		icm42688_accel_ms(&ddata->cfg, raw_to_cpu(data[6], data[7]),  &ms[2], &ums[2]);

		/* causes a shift overflow on the 1000000 value when doing INT_TO_FP()  */

		/*
		adata->readings[0].x = INT_TO_FP(ms[0]) + fp_div(INT_TO_FP(ums[0]), INT_TO_FP(1000000));
		adata->readings[0].y = INT_TO_FP(ms[0]) + fp_div(INT_TO_FP(ums[0]), INT_TO_FP(1000000));
		adata->readings[0].z = INT_TO_FP(ms[0]) + fp_div(INT_TO_FP(ums[0]), INT_TO_FP(1000000));
		*/
		
		/* FP doesn't have enough precision in fixed point to deal with this math */
		/* Float results in 0's which seems wrong */
                /* Negatives don't work as the ms value + ums should be - if the sign of ms is negative. */
		adata->readings[0].x = FLOAT_TO_FP((float)ms[0] + ((float)ums[0])/1000000.0f);
		adata->readings[0].y = FLOAT_TO_FP((float)ms[1] + ((float)ums[1])/1000000.0f);
		adata->readings[0].z = FLOAT_TO_FP((float)ms[2] + ((float)ums[2])/1000000.0f);

	} else if (sensor_type == SENSOR_TYPE_GYROSCOPE) {
		struct sensor_three_axis_data *gdata = sdata;
		int32_t rads[3];
		uint32_t urads[3];		

		icm42688_gyro_rads(&ddata->cfg, raw_to_cpu(data[8], data[9]),  &rads[0], &urads[0]);
		icm42688_gyro_rads(&ddata->cfg, raw_to_cpu(data[10], data[11]),  &rads[1], &urads[1]);
		icm42688_gyro_rads(&ddata->cfg, raw_to_cpu(data[12], data[13]),  &rads[2], &urads[2]);

		/* causes a shift overflow on the 1000000 value when doing INT_TO_FP and CONFIG_FPU=n */
		/*
		gdata->readings[0].x = INT_TO_FP(rads[0]) + fp_div(INT_TO_FP(urads[0]), INT_TO_FP(1000000));
		gdata->readings[0].y = INT_TO_FP(rads[1]) + fp_div(INT_TO_FP(urads[1]), INT_TO_FP(1000000));
		gdata->readings[0].z = INT_TO_FP(rads[2]) + fp_div(INT_TO_FP(urads[2]), INT_TO_FP(1000000));
		*/

		gdata->readings[0].x = FLOAT_TO_FP((float)rads[0] + (float)rads[0]/1000000.0f);
		gdata->readings[0].y = FLOAT_TO_FP((float)rads[1] + (float)rads[1]/1000000.0f);
		gdata->readings[0].z = FLOAT_TO_FP((float)rads[2] + (float)rads[2]/1000000.0f);
		
	} else if (sensor_type == SENSOR_TYPE_ACCELEROMETER_TEMPERATURE
		|| sensor_type == SENSOR_TYPE_GYROSCOPE_TEMPERATURE) {
		
		struct sensor_float_data *tdata = sdata;
		int32_t c;
		uint32_t uc;

		icm42688_temp_c(raw_to_cpu(data[0], data[1]), &c, &uc);

		tdata->readings[0].value = FLOAT_TO_FP((float)c + (float)uc/1000000.0f);
	} else {
		res = -ENOTSUP;
	}
	
out:
	return res;
}


static inline int icm42688_accel_range(enum icm42688_accel_fs fs)
{
	int ret = 0;
	
	switch (fs) {
		case ICM42688_ACCEL_FS_16G:
			ret = 16;
			break;
		case ICM42688_ACCEL_FS_8G:
			ret = 8;
			break;
		case ICM42688_ACCEL_FS_4G:
			ret = 4;
			break;
		case ICM42688_ACCEL_FS_2G:
			ret = 2;
			break;
	}	
	
	return ret;
}

static inline int icm42688_gyro_range_x1000(enum icm42688_gyro_fs fs)
{
	int ret;
	
	switch (fs) {
		case ICM42688_GYRO_FS_2000:
			ret = 2000000;
			break;
		case ICM42688_GYRO_FS_1000:
			ret = 1000000;
			break;
		case ICM42688_GYRO_FS_500:
			ret = 500000;
			break;
		case ICM42688_GYRO_FS_250:
			ret = 250000;
			break;
		case ICM42688_GYRO_FS_125:
			ret = 125000;
			break;
		case ICM42688_GYRO_FS_62_5:
			ret = 62500;
			break;
		case ICM42688_GYRO_FS_31_25:
			ret = 31250;
			break;
		case ICM42688_GYRO_FS_15_625:
			ret = 15625;
			break;
	}
	
	return ret;
}



static int icm42688_get_scale(const struct device *dev, uint32_t sensor_type, struct sensor_scale_metadata *scale)
{
	int res = 0;
	struct icm42688_dev_data *ddata = dev->data;

	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		scale->range_units = SENSOR_RANGE_UNITS_ACCEL_G;
		scale->range = INT_TO_FP(icm42688_accel_range(ddata->cfg.accel_fs));
		// TODO this doesn't work, just because the FIFO is enabled doesn't mean someone can't read_all
		// and the resolution is possibly different
		scale->resolution = 16;
	} else if (sensor_type == SENSOR_TYPE_GYROSCOPE) {
		// TODO deg/s and rad/s not just degrees or radians
		scale->range_units = SENSOR_RANGE_UNITS_ANGLE_DEGREES;
		scale->range = FLOAT_TO_FP((float)icm42688_gyro_range_x1000(ddata->cfg.gyro_fs)/1000.0);
		// TODO this doesn't work, just because the FIFO is enabled doesn't mean someone can't read_all
		// and the resolution is possibly different
		scale->resolution = 16;
	} else if (sensor_type == SENSOR_TYPE_ACCELEROMETER_TEMPERATURE ||
		   sensor_type == SENSOR_TYPE_GYROSCOPE_TEMPERATURE) {
		scale->range_units = SENSOR_RANGE_UNITS_TEMPERATURE_C;
		scale->range = INT_TO_FP(100); /* datasheet gives sensitivity (change per bit) and offset, just assume 100C */
		// TODO if temp data is from the fifo the resolution is 8 bits, from the data registers its 16 bits
		scale->resolution = 16;
	} else {
		res = -ENOTSUP;
	}
	
	return res;
}

static inline int icm42688_accel_range_to_fs(uint32_t range, bool round_up, enum icm42688_accel_fs *fs)
{
	int ret;
	
	if (range == 16 || (round_up && range > 8)) {
		*fs = ICM42688_ACCEL_FS_16G;
	} else if (range == 8 || (round_up && range > 4)) {
		*fs = ICM42688_ACCEL_FS_8G ;
	} else if (range == 4 || (round_up && range > 2)) {
		*fs = ICM42688_ACCEL_FS_4G;
	} else if (range == 2 || (round_up && range > 0)) {
		*fs = ICM42688_ACCEL_FS_2G;
	} else {
		ret = -1;
	}
	
	return ret;
}

static inline int icm42688_gyro_range_to_fs(uint32_t range, bool round_up, enum icm42688_gyro_fs *fs)
{
	int ret;
	
	if (range == 2000 || (round_up && range > 1000)) {
		*fs = ICM42688_GYRO_FS_2000;
	} else if (range == 1000 || (round_up && range > 500)) {
		*fs = ICM42688_GYRO_FS_1000;
	} else if (range == 500 || (round_up && range > 250)) {
		*fs = ICM42688_GYRO_FS_500;
	} else if (range == 250 || (round_up && range > 125)) {
		*fs = ICM42688_GYRO_FS_250;
	} else if (range == 125 || (round_up && range > 63)) {
		*fs = ICM42688_GYRO_FS_125;
	} else if (range == 62 || range == 63 || (round_up && range > 32)) {
		*fs = ICM42688_GYRO_FS_62_5;
	} else if (range == 31 || range == 32 || (round_up && range > 16)) {
		*fs = ICM42688_GYRO_FS_31_25;
	} else if (range == 15 || range == 16 || (round_up && range > 0)) {
		*fs = ICM42688_GYRO_FS_15_625;
	} else {
		ret = -1;
	}
	
	return ret;
}

int icm42688_set_range(const struct device *dev, uint32_t sensor_type,
	fp_t range, bool round_up)
{
	int res;
	struct icm42688_sensor_data *data = dev->data;
	uint32_t range_i = FP_TO_INT(range);
	struct icm42688_cfg mcfg = data->dev_data.cfg;
	
	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		enum icm42688_accel_fs fs;

		res = icm42688_accel_range_to_fs(range_i, round_up, &fs);
		if (res != 0) {
			goto out;
		}
		mcfg.accel_fs = fs;
	} else if(sensor_type == SENSOR_TYPE_GYROSCOPE) {
		enum icm42688_gyro_fs fs;

		res = icm42688_gyro_range_to_fs(range_i, round_up, &fs);
		if (res != 0) {
			goto out;
		}
		mcfg.gyro_fs = fs;
	} else {
		res = -ENOTSUP;
		goto out;
	}

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

out:
	return res;
}


int icm42688_set_resolution(const struct device *dev, uint32_t sesnor_type, uint8_t resolution, bool round_up)
{
	/* TODO streaming mode allows up to 20bits, without streaming mode 16 bits, not sure how to implement this
	   as streaming mode is mutable elsewhere... very confused
	 */ 
	
	return -ENOTSUP;
}

int icm42688_get_bias(const struct device *dev, uint32_t sensor_type, int16_t *temperature,
		      fp_t *bias_x, fp_t *bias_y, fp_t *bias_z)
{

	/* TODO get bias values */
	return -ENOTSUP;	
}


int icm42688_set_bias(const struct device *dev, uint32_t sensor_type, int16_t temperature,
		      fp_t bias_x, fp_t bias_y, fp_t bias_z, bool round_up)
{

	/* TODO set bias values */
	return -ENOTSUP;	
}


static const struct sensor_sample_rate_info icm42688_sample_rates[27] = 
	{
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 32000000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 16000000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 8000000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 4000000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 2000000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 1000000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 500000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 200000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 100000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 50000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 25000,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 12500,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 6250,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 3125,
		},
		{
			.sensor_type = SENSOR_TYPE_ACCELEROMETER,
			.sample_rate_mhz = 1562, /* actually 1.5625 Hz */
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 32000000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 16000000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 8000000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 4000000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 2000000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 1000000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 500000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 200000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 100000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 50000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 25000,
		},
		{
			.sensor_type = SENSOR_TYPE_GYROSCOPE,
			.sample_rate_mhz = 12500,
		},

};

/** This is a nice introspection function, maybe range should also have something like this? Does this apply to all sensors? */
int icm42688_get_sample_rate_available(const struct device *dev, const struct sensor_sample_rate_info **sample_rates,
	uint8_t *count)
{
	int res = 0;

	*sample_rates = icm42688_sample_rates;
	*count = ARRAY_SIZE(icm42688_sample_rates);
		
	return res;
}


int icm42688_get_sample_rate(const struct device *dev, uint32_t sensor_type,
	uint32_t *sample_rate)
{
	int res = 0;

	/* TODO get sample rate */	
		
	return res;
}

static inline int icm42688_accel_rate_to_odr(uint32_t sample_rate, bool round_up, enum icm42688_accel_odr *odr)
{
	int ret;
	
	if (sample_rate == 32000 || (round_up && sample_rate > 16000)) {
		*odr = ICM42688_ACCEL_ODR_32000;
	} else if (sample_rate == 16000 || (round_up && sample_rate > 8000)) {
		*odr = ICM42688_ACCEL_ODR_16000;
	} else if (sample_rate == 8000 || (round_up && sample_rate > 4000)) {
		*odr = ICM42688_ACCEL_ODR_8000;
	} else if (sample_rate == 4000 || (round_up && sample_rate > 2000)) {
		*odr = ICM42688_ACCEL_ODR_4000;
	} else if (sample_rate == 2000 || (round_up && sample_rate > 1000)) {
		*odr = ICM42688_ACCEL_ODR_2000;
	} else if (sample_rate == 1000 || (round_up && sample_rate > 500)) {
		*odr = ICM42688_ACCEL_ODR_1000;
	} else if (sample_rate == 500 || (round_up && sample_rate > 200)) {
		*odr = ICM42688_ACCEL_ODR_500;
	} else if (sample_rate == 200 || (round_up && sample_rate > 100)) {
		*odr = ICM42688_ACCEL_ODR_200;
	} else if (sample_rate == 100 || (round_up && sample_rate > 50)) {
		*odr = ICM42688_ACCEL_ODR_100;
	} else if (sample_rate == 50 || (round_up && sample_rate > 25)) {
		*odr = ICM42688_ACCEL_ODR_50;
	} else if (sample_rate == 25 || (round_up && sample_rate > 13)) {
		*odr = ICM42688_ACCEL_ODR_25;
	} else if (sample_rate == 12 || sample_rate == 13 || (round_up && sample_rate > 7)) {
		*odr = ICM42688_ACCEL_ODR_12_5;
	} else if (sample_rate == 6 || sample_rate == 7 || (round_up && sample_rate > 4)) {
		*odr = ICM42688_ACCEL_ODR_6_25;
	} else if (sample_rate == 3 || sample_rate == 4 ||  (round_up && sample_rate > 2)) {
		*odr = ICM42688_ACCEL_ODR_3_125;
	} else if (sample_rate == 1 || sample_rate == 2 || (round_up && sample_rate > 0)) {
		*odr = ICM42688_ACCEL_ODR_1_5625;
	} else {
		ret = -1;
	}
	
	return ret;
}


static inline int icm42688_gyro_rate_to_odr(uint32_t sample_rate, bool round_up, enum icm42688_gyro_odr *odr)
{
	int ret;
	
	if (sample_rate == 32000 || (round_up && sample_rate > 16000)) {
		*odr = ICM42688_GYRO_ODR_32000;
	} else if (sample_rate == 16000 || (round_up && sample_rate > 8000)) {
		*odr = ICM42688_GYRO_ODR_16000;
	} else if (sample_rate == 8000 || (round_up && sample_rate > 4000)) {
		*odr = ICM42688_GYRO_ODR_8000;
	} else if (sample_rate == 4000 || (round_up && sample_rate > 2000)) {
		*odr = ICM42688_GYRO_ODR_4000;
	} else if (sample_rate == 2000 || (round_up && sample_rate > 1000)) {
		*odr = ICM42688_GYRO_ODR_2000;
	} else if (sample_rate == 1000 || (round_up && sample_rate > 500)) {
		*odr = ICM42688_GYRO_ODR_1000;
	} else if (sample_rate == 500 || (round_up && sample_rate > 200)) {
		*odr = ICM42688_GYRO_ODR_500;
	} else if (sample_rate == 200 || (round_up && sample_rate > 100)) {
		*odr = ICM42688_GYRO_ODR_200;
	} else if (sample_rate == 100 || (round_up && sample_rate > 50)) {
		*odr = ICM42688_GYRO_ODR_100;
	} else if (sample_rate == 50 || (round_up && sample_rate > 25)) {
		*odr = ICM42688_GYRO_ODR_50;
	} else if (sample_rate == 25 || (round_up && sample_rate > 13)) {
		*odr = ICM42688_GYRO_ODR_25;
	} else if (sample_rate == 12 || sample_rate == 13 || (round_up && sample_rate > 0)) {
		*odr = ICM42688_GYRO_ODR_12_5;
	} else {
		ret = -1;
	}
	
	return ret;
}

int icm42688_set_sample_rate(const struct device *dev, uint32_t sensor_type,
	uint32_t sample_rate, bool round_up)
{
	struct icm42688_sensor_data *data = dev->data;
	
	int res;

	/* copy configuration */
	struct icm42688_cfg mcfg = data->dev_data.cfg;
	
	if (sensor_type == SENSOR_TYPE_ACCELEROMETER) {
		enum icm42688_accel_odr odr;

		res = icm42688_accel_rate_to_odr(sample_rate, round_up, &odr);
		if (res != 0) {
			goto out;
		}
		mcfg.accel_odr = odr;
	} else if(sensor_type == SENSOR_TYPE_GYROSCOPE) {
		enum icm42688_gyro_odr odr;

		res = icm42688_gyro_rate_to_odr(sample_rate, round_up, &odr);
		if (res != 0) {
			goto out;
		}
		mcfg.gyro_odr = odr;
	} else {
		res = -ENOTSUP;
		goto out;
	}

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

out:
	return res;
}

int icm42688_set_fifo_data_buffer(const struct device *dev, 
				  struct sensor_raw_data *buffer)
{
	struct icm42688_sensor_data *data = dev->data;

	/* Possible hazard here, if fifo's are already enabled, and the buffer is already set
	 * and we are in the callback, we need to lock around the process callback
	 * but that is... not very nice */
	data->fifo_buf = buffer;
	
		
	return 0;
}

int icm42688_set_process_data_callback(const struct device *dev,
				       sensor_process_data_callback_t callback)
{
	struct icm42688_sensor_data *data = dev->data;

	
	/* This should be pretty safe without a lock */
	data->process_callback = callback;
			
	return 0;
}

int icm42688_flush_fifo(const struct device *dev)
{
	struct icm42688_sensor_data *data = dev->data;

	/* Reconfiguring the sensor flushes the FIFO */
	return icm42688_configure(dev, &data->dev_data.cfg);
}


int icm42688_iter_next(struct sensor_fifo_iterator *iter)
{
	if(iter->data->header.reading_count*4 < iter->offset) {
		return -EINVAL;
	}

	if (iter->offset == 0 && iter->data->header.reading_count > 0) {
		iter->offset++;
	}
	return 0;
}

/* This isn't that nice to write honestly */
int icm42688_iter_sensor_type(const struct sensor_fifo_iterator *iter, uint32_t *sensor_type)
{
	if(iter->data->header.reading_count < iter->offset) {
		return -EINVAL;
	}

	int sample_idx = (iter->offset - 1)/4;
	int type_pos = (iter->offset - 1) - sample_idx;
	
	switch (type_pos) {
		case 0:
			*sensor_type = SENSOR_TYPE_ACCELEROMETER;
			break;
		case 1:
			*sensor_type = SENSOR_TYPE_GYROSCOPE;
			break;
		case 2:
			*sensor_type = SENSOR_TYPE_ACCELEROMETER_TEMPERATURE;
			break;
		/* FIXME */
		/*
		case 3:
			*sensor_type = SENSOR_TYPE_TIMESTAMP;
			break;
		*/
	}

	return 0;
}

int icm42688_iter_read(const struct sensor_fifo_iterator *iter, void *out)
{
	int sample_idx = (iter->offset - 1)/4;
	int type_pos = (iter->offset - 1) - sample_idx;

	/* This really should depend more on the FIFO format and sensor configuration.
	   The gyro/accel may be enabled/disabled, 20 bit may be enabled/disabled.
	   There are 4 possible layouts. The layout is self described in each packet
	   header.
	 */

	switch (type_pos) {
		case 0:
			/* take byte position (idx-1) + [1:6] as accelerometer data */
			break;
		case 1:
			/* take byte position (idx-1) + [7:13]  as gyroscope data */
			break;
		case 2:
			/* take byte position (idx-1) + [14] as temperature data */
			break;
		case 3:
			/* take byte position (idx-1) + [15:16] as timestamp data */
			break;
	}

	
	return 0;
}

int icm42688_get_fifo_iterator(const struct device *dev,
			       struct sensor_fifo_iterator_api *iter_api)
{
	/* No way to attach the configuration of the sensor currently for an iterator
	 * so it can't outlive the buffer... different callbacks could be given
	 * depending on the config? maybe that's the intent?
	 */
	iter_api->next = icm42688_iter_next;
	iter_api->get_sensor_type = icm42688_iter_sensor_type;
	iter_api->read = icm42688_iter_read;
	
	return 0;
		
}

int icm42688_get_watermark(const struct device *dev,
			   uint8_t *wm_pct)

{
	
	struct icm42688_sensor_data *data = dev->data;
	
	return -ENOTSUP;
}

int icm42688_set_watermark(const struct device *dev,
			   uint8_t wm_pct,
			   bool round_up)

{
	struct icm42688_sensor_data *data = dev->data;
	int res;
	
	/* copy configuration */
	struct icm42688_cfg mcfg = data->dev_data.cfg;

	const int FIFO_SIZE = 2048;
	const int HIRES_PKT = 20;
	const int PKT = 16;

	__ASSERT(pct <= 100 && pct >= 0, "watermark percentage should be in range 0 to 100");

	int pkt_sz = mcfg.fifo_hires ? HIRES_PKT : PKT;
	int n_pkts = FIFO_SIZE/pkt_sz;
	int wm = (n_pkts*wm_pct)/(100);
	
	mcfg.fifo_wm = wm;
	
	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);

out:
	return res;		
}

int icm42688_get_streaming_mode(const struct device *dev,
				bool *enabled)
{

	struct icm42688_sensor_data *data = dev->data;

	*enabled = data->dev_data.cfg.fifo_en;
	
	return 0;
}

int icm42688_set_streaming_mode(const struct device *dev,
				bool enabled)
{
	struct icm42688_sensor_data *data = dev->data;
	const struct icm42688_sensor_config *cfg = dev->cfg;
	int res;

	/* copy configuration */
	struct icm42688_cfg mcfg = data->dev_data.cfg;
	
	/* TODO Might be cool to enable this */
	mcfg.fifo_hires = false;
	mcfg.fifo_en = enabled;

	/* Enable the gpio irq */
	gpio_pin_interrupt_configure_dt(&cfg->dev_cfg.gpio_int1, GPIO_INT_EDGE_TO_ACTIVE);

	/* reconfigure sensor, making the modified config the new sensor config if valid */
	res = icm42688_configure(dev, &mcfg);
		

out:
	return res;		
}

int icm42688_perform_calibration(const struct device *dev,
				bool enabled)
{
	return -ENOTSUP;
}

/**
 * Thread waiting on sensor events
 */
static void icm42688_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	
	struct device *dev = p1;
	struct icm42688_sensor_data *data = dev->data;
	const struct icm42688_sensor_config *cfg = dev->cfg;
	
	while (1) {
		k_sem_take(&data->int1_sem, K_FOREVER);
		/* Read the interrupt cause */
		
		/* If cause is FIFO FULL or Watermark then... */

		/* Read fifo length then... */

		/* Read fifo into struct sensor_raw_data then... */
		//icm42688_spi_read()

		/* Call the sensor process callback */
	}
}

/** This is unfortunately needed */
static void icm42688_gpio_callback(const struct device *dev,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct icm42688_sensor_data *data =
		CONTAINER_OF(cb, struct icm42688_sensor_data, gpio_cb);

	ARG_UNUSED(pins);

	k_sem_give(&data->int1_sem);
}

static int icm42688_init(const struct device *dev)
{
	struct icm42688_sensor_data *data = dev->data;
	const struct icm42688_sensor_config *cfg = dev->config;
	int res;

	if (!spi_is_ready(&cfg->dev_cfg.spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}
		
	if (!device_is_ready(cfg->dev_cfg.gpio_int1.port)) {
		LOG_ERR("gpio_int1 gpio not ready");
		return -ENODEV;
	}
	
	gpio_pin_configure_dt(&cfg->dev_cfg.gpio_int1, GPIO_INPUT);
	gpio_init_callback(&data->gpio_cb1, icm42688_gpio_callback, BIT(cfg->dev_cfg.gpio_int1.pin));
	res = gpio_add_callback(cfg->dev_cfg.gpio_int1.port, &data->gpio_cb1);
	
	if (res != 0) {
		LOG_ERR("could not configure gpio callback");
		return res;
	}
	
	k_sem_init(&data->int1_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
		CONFIG_ICM42688_THREAD_STACK_SIZE, icm42688_thread, dev, NULL, NULL,
		K_PRIO_COOP(CONFIG_ICM42688_THREAD_PRIORITY), 0, K_NO_WAIT);
	
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_INACTIVE);

	res = icm42688_reset(dev);
	if (res != 0) {
		LOG_ERR("could not initialize sensor");
		return res;
	}

	// TODO interpret the config params from DT here using the X to Y conversions.
	data->dev_data.cfg.accel_mode = ICM42688_ACCEL_LN;
	data->dev_data.cfg.gyro_mode = ICM42688_GYRO_LN;
	data->dev_data.cfg.accel_fs = ICM42688_ACCEL_FS_16G;
	data->dev_data.cfg.gyro_fs = ICM42688_GYRO_FS_2000;
	data->dev_data.cfg.accel_odr = ICM42688_ACCEL_ODR_32000;
	data->dev_data.cfg.gyro_odr = ICM42688_GYRO_ODR_32000;

	res = icm42688_configure(dev, &data->dev_data.cfg);
	
	if (res != 0) {
		LOG_ERR("could not configure sensor");
	}
	
	return res;
}

static const struct sensor_driver_api_v2 icm42688_driver_api = {
	.read_data = icm42688_read_data,
	.get_scale = icm42688_get_scale,
	.set_range = icm42688_set_range,
	.get_bias = icm42688_get_bias,
	.set_bias = icm42688_set_bias,
#ifdef CONFIG_SENSOR_STREAMING_MODE
	set_fifo_data_buffer = icm42688_set_fifo_data_buffer,
	.set_process_data_callback = icm42688_set_process_data_callback,
	.flush_fifo = icm42688_flush_fifo,
	.get_fifo_iterator_api = icm42688_get_fifo_iterator,
	.get_sample_rate_available = icm42688_get_sample_rate_available,
	.get_sample_rate = icm42688_get_sample_rate,
	.set_sample_rate = icm42688_set_sample_rate,
	.get_watermark = icm42688_get_watermark,
	.set_watermark = icm42688_set_watermark,
	.get_streaming_mode = icm42688_get_streaming_mode,
	.set_streaming_mode = icm42688_set_streaming_mode,
#endif /* CONFIG_SENSOR_STREAMING_MODE */
#ifdef CONFIG_SENSOR_HW_CALIBRATION
.perform_calibration = icm42688_perform_calibration,
#endif
};

/* device defaults to spi mode 0/3 support */
#define ICM42688_SPI_CFG                                                                           \
	SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define ICM42688_INIT(inst)                                                                        \
	static struct icm42688_sensor_data icm42688_driver_##inst = {                              \
	};                                                                                         \
												   \
	static const struct icm42688_sensor_config icm42688_cfg_##inst = {                         \
		.dev_cfg = {									   \
			.spi = SPI_DT_SPEC_INST_GET(inst, ICM42688_SPI_CFG, 0U),                   \
		},										   \
	};                                                                                         \
												   \
	DEVICE_DT_INST_DEFINE(inst, icm42688_init, NULL, &icm42688_driver_##inst,                  \
			      &icm42688_cfg_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,      \
			      &icm42688_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM42688_INIT)
