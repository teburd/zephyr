/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <sensor.h>
#include <stdio.h>
#include <misc/util.h>

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static int print_samples;
static int lis3mdl_trig_cnt;

static struct sensor_value magn_x_out, magn_y_out, magn_z_out;

#ifdef CONFIG_LIS3MDL_TRIGGER
static void lis3mdl_trigger_handler(struct device *dev,
				    struct sensor_trigger *trig)
{
	static struct sensor_value magn_x, magn_y, magn_z;
	lis3mdl_trig_cnt++;

	/* lis3mdl magn */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);


	if (print_samples) {
		print_samples = 0;
		magn_x_out = magn_x;
		magn_y_out = magn_y;
		magn_z_out = magn_z;
	}

}
#endif

void main(void)
{
	int cnt = 0;
	char out_str[64];
	struct device *lis3mdl_dev = device_get_binding(DT_ST_LIS3MDL_0_LABEL);

	if (lis3mdl_dev == NULL) {
		printk("Could not get LIS3MDL device\n");
		return;
	}

#ifdef CONFIG_LIS3MDL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lis3mdl_dev, &trig, lis3mdl_trigger_handler);
#endif

	if (sensor_sample_fetch(lis3mdl_dev) < 0) {
		printk("Sensor sample update error\n");
		return;
	}

	while (1) {
		/* Erase previous */
		printk("\0033\014");
		printf("LIS3MDL sensor samples:\n\n");

		/* lis3mdl external magn */
		sprintf(out_str, "magn (%f %f %f) gauss", out_ev(&magn_x_out),
							 out_ev(&magn_y_out),
							 out_ev(&magn_z_out));
		printk("%s\n", out_str);

		printk("- (%d) (trig_cnt: %d)\n\n", ++cnt, lis3mdl_trig_cnt);
		print_samples = 1;
		k_sleep(2000);
	}
}
