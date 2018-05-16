#include <spi.h>
#include <init.h>
#include <sensor.h>
#include "icm20649.h"

#define SENSOR_PI_DOUBLE			(SENSOR_PI / 1000000.0)
#define SENSOR_DEG2RAD_DOUBLE			(SENSOR_PI_DOUBLE / 180)
#define SENSOR_G_DOUBLE				(SENSOR_G / 1000000.0)

#if defined(CONFIG_ICM20649_SPI_GPIO_CS)
static struct spi_cs_control icm20649_cs_ctrl;
#endif

#define SPI_CS NULL

static struct spi_config icm20649_spi_conf = {
	.frequency = CONFIG_ICM20649_SPI_BUS_FREQ,
	.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
		      SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE),
	.slave     = CONFIG_ICM20649_SPI_SELECT_SLAVE,
	.cs        = SPI_CS,
};



#define ICM20649_MAX_SERIAL_READ 16

static int icm20649_raw_read(struct icm20649_data *data, u8_t reg_addr, u8_t *value, u8_t len) {
	struct spi_config *spi_cfg = &icm20649_spi_conf;
	u8_t buffer_tx[2] = { reg_addr | ICM20649_SPI_READ, 0};

	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (len > ICM20649_MAX_SERIAL_READ) {
		return -EIO;
	}

	if (spi_transceive(data->spi, spi_cfg, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

static int icm20649_raw_write(struct icm20649_data *data, u8_t reg_addr,
		u8_t *value, u8_t len)
{
	struct spi_config *spi_cfg = &icm20649_spi_conf;
	u8_t buffer_tx[1] = { reg_addr & ~ICM20649_SPI_READ };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	if (len > 64) {
		return -EIO;
	}

	if (spi_write(data->spi, spi_cfg, &tx)) {
		return -EIO;
	}

	return 0;
}

static inline int icm20649_write_reg8(struct device *dev, u8_t addr, u8_t val) {
	struct icm20649_data *data = dev->driver_data;
	return icm20649_raw_write(data, addr, &val, 1);
}

static inline int icm20649_read_reg8(struct device *dev, u8_t addr, u8_t *val) {
	struct icm20649_data *data = dev->driver_data;
	return icm20649_raw_read(data, addr, val, 1);
}

static inline int icm20649_read_s16(struct device *dev, u8_t addr_l, u8_t addr_h, s16_t *val) {
	u8_t l, h = 0;
	int err = icm20649_read_reg8(dev, addr_l, &l);
	err |= icm20649_read_reg8(dev, addr_h, &h);
	*val = (s16_t)((u16_t)(l) |
				((u16_t)(h) << 8));
	return err;
}

static inline int icm20649_read_u16(struct device *dev, u8_t addr_h, u8_t addr_l, u16_t *val) {
	u8_t l, h = 0;
	int err = icm20649_read_reg8(dev, addr_h, &h);
	err |= icm20649_read_reg8(dev, addr_l, &l);
	*val = ((u16_t)(l) |
				((u16_t)(h) << 8));
	return err;
}

static inline int icm20649_update_reg8(struct device *dev, u8_t addr, u8_t mask,
		u8_t val) {
	struct icm20649_data *data = dev->driver_data;

	u8_t tmp_val;
	
	int ret = icm20649_raw_read(data, addr, &tmp_val, 1);
	tmp_val = (tmp_val & ~mask) | (val & mask);

	ret |= icm20649_raw_write(data, addr, &tmp_val, 1);

#ifdef CONFIG_ICM20649_VERIFY_WRITE
	u8_t new_val;
	ret |= icm20649_raw_read(data, addr, &new_val, 1);
	if(new_val != tmp_val) {
		SYS_LOG_DBG("failed to verify write for addr %x expected %x got %x", addr, tmp_val, new_val);
	}
#endif
	return ret;
}

static inline int icm20649_set_user_bank(struct device *dev, u8_t bank) {
	return icm20649_write_reg8(dev, ICM20649_REG_BANK_SEL,
			bank << ICM20649_SHIFT_BANK_SEL & ICM20649_MASK_BANK_SEL);
}

static inline int icm20649_reset(struct device *dev) {
	SYS_LOG_DBG("reset");
	int ret = icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			ICM20649_MASK_PWR_MGMT_1_DEVICE_RESET,
			(1 << ICM20649_SHIFT_PWR_MGMT_1_DEVICE_RESET));
	SYS_LOG_DBG("reset done");
	return ret;
}

static inline int icm20649_auto_clock(struct device *dev) {
	SYS_LOG_DBG("auto clock");
	int ret = icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			ICM20649_MASK_PWR_MGMT_1_CLKSEL,
			(1 << ICM20649_SHIFT_PWR_MGMT_1_CLKSEL));
	SYS_LOG_DBG("auto clock done");
	return ret;
}

static inline int icm20649_wakeup(struct device *dev) {
	SYS_LOG_DBG("wakeup");
	int ret = icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1,
			ICM20649_MASK_PWR_MGMT_1_SLEEP,
			0 << ICM20649_SHIFT_PWR_MGMT_1_SLEEP);
	SYS_LOG_DBG("wakeup done");
	return ret;
}

static inline int icm20649_user_config(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_USER_CTRL,
			ICM20649_MASK_USER_CTRL_DMP_EN |
			ICM20649_MASK_USER_CTRL_FIFO_EN |
			ICM20649_MASK_USER_CTRL_I2C_IF_DIS ,
			0 << ICM20649_SHIFT_USER_CTRL_DMP_EN |
			1 << ICM20649_SHIFT_USER_CTRL_FIFO_EN |
			1 << ICM20649_SHIFT_USER_CTRL_I2C_IF_DIS
			);
}

static inline int icm20649_sleep(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			ICM20649_MASK_PWR_MGMT_1_SLEEP,
			(1 << ICM20649_SHIFT_PWR_MGMT_1_SLEEP));
}

int icm20649_get_who_am_i(struct device *dev, u8_t *whoami) {
	return icm20649_read_reg8(dev, ICM20649_REG_WHO_AM_I, whoami);
}

void icm20649_fifo_stream(struct device *dev, icm20649_fifo_stream_cb stream_cb)
{
}

static inline int icm20649_lp_enable(struct device *dev) {
	int ret = icm20649_update_reg8(dev, ICM20649_REG_LP_CONFIG,
            ICM20649_MASK_LP_CONFIG_ACCEL_CYCLE | 
            ICM20649_MASK_LP_CONFIG_GYRO_CYCLE,
            1 << ICM20649_SHIFT_LP_CONFIG_ACCEL_CYCLE |
            1 << ICM20649_SHIFT_LP_CONFIG_GYRO_CYCLE
            );
	ret |= icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_1,
			ICM20649_MASK_PWR_MGMT_1_LP_EN,
			0 << ICM20649_SHIFT_PWR_MGMT_1_LP_EN);
	return ret;
}

static inline int icm20649_accel_gyro_enable(struct device *dev) {
    return icm20649_update_reg8(dev, ICM20649_REG_PWR_MGMT_2,
            ICM20649_MASK_PWR_MGMT_2_DISABLE_ACCEL | 
            ICM20649_MASK_PWR_MGMT_2_DISABLE_GYRO,
            0 << ICM20649_SHIFT_PWR_MGMT_2_DISABLE_ACCEL |
            0 << ICM20649_SHIFT_PWR_MGMT_2_DISABLE_GYRO
            );

}

static inline int icm20649_set_accel_sample_rate_div(struct device *dev,
		u16_t rate_div) {
	int ret = icm20649_write_reg8(dev, ICM20649_REG_ACCEL_SMPLRT_DIV_2, (u8_t)(rate_div & 0x00FF));
	ret |= icm20649_write_reg8(dev, ICM20649_REG_ACCEL_SMPLRT_DIV_1, (u8_t)(rate_div >> 8 & ICM20649_MASK_ACCEL_SMPLRT_DIV_1));
	return ret;
}

static inline int icm20649_set_accel_decimator(struct device *dev, u8_t avg) {
	u8_t dec2;
	switch(avg) {
		case 4:
			dec2 = 0;
			break;
		case 8:
			dec2 = 1;
			break;
		case 16:
			dec2 = 2;
			break;
		case 32:
			dec2 = 3;
			break;
		default:
			SYS_LOG_DBG("invalid number of samples to average, must be 4, 8, 16, or 32");
			return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG_2,
			ICM20649_MASK_ACCEL_CONFIG_2_DEC3_CFG,
			dec2 << ICM20649_SHIFT_ACCEL_CONFIG_2_DEC3_CFG);
}

static inline int icm20649_set_accel_filter_choice(struct device *dev,
		u8_t fchoice)
{
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG,
			ICM20649_MASK_ACCEL_CONFIG_ACCEL_FCHOICE,
		fchoice << ICM20649_SHIFT_ACCEL_CONFIG_ACCEL_FCHOICE);
}

static inline int icm20649_set_accel_lpf(struct device *dev, u8_t lpf_mode) {
	if(lpf_mode > 7) {
		SYS_LOG_DBG("invalid accelerometer low pass filter mode, must be 0 to 7");
		return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG,
			ICM20649_MASK_ACCEL_CONFIG_ACCEL_DLPFCFG,
			lpf_mode << ICM20649_SHIFT_ACCEL_CONFIG_ACCEL_DLPFCFG);
}

static int icm20649_set_accel_fs_sel(struct device *dev, u8_t fs) {
	u8_t update = 0;
	if(fs > 3) {
		SYS_LOG_DBG("invalid accelerometer full-scale must be 0 to 3");
		return -EINVAL;
	}
	update |= fs << ICM20649_SHIFT_ACCEL_CONFIG_ACCEL_FS_SEL;
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG,
			ICM20649_MASK_ACCEL_CONFIG_ACCEL_FS_SEL, update);
}

static inline int icm20649_set_gyro_filter_choice(struct device *dev, u8_t fchoice) {
	if(fchoice > 3) {
		SYS_LOG_DBG("invalid gyro filter choice, must be 0 to 3");
		return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			ICM20649_MASK_GYRO_CONFIG_1_GYRO_FCHOICE,
			fchoice << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FCHOICE);
}

static inline int icm20649_set_gyro_lpf(struct device *dev, u8_t lpf_mode) {
	if(lpf_mode > 7) {
		SYS_LOG_DBG("invalid gyro low pass filter mode, must be 0 to 7");
		return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			ICM20649_MASK_GYRO_CONFIG_1_GYRO_DLPFCFG,
			lpf_mode << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_DLPFCFG);
}

static int icm20649_set_gyro_fs_sel(struct device *dev, u8_t fs) {
	u8_t update = 0;
	if(fs > 3) {
		SYS_LOG_DBG("invalid gyro full-scale must be 0 to 3");
		return -EINVAL;
	}
	update |= fs << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FS_SEL;
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			ICM20649_MASK_GYRO_CONFIG_1_GYRO_FS_SEL, update);
}

static inline int icm20649_set_gyro_sample_rate_div(struct device *dev,
		u8_t rate_div) {
	return icm20649_write_reg8(dev, ICM20649_REG_GYRO_SMPLRT_DIV, rate_div);
}

static int icm20649_set_odr_align(struct device *dev, u8_t align) {
	return icm20649_update_reg8(dev, ICM20649_REG_ODR_ALIGN_EN,
			ICM20649_MASK_ODR_ALIGN_EN,
			align << ICM20649_SHIFT_ODR_ALIGN_EN);
}

static int icm20649_attr_set(struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	SYS_LOG_WRN("attr_set() not supported on this channel.");
	return -ENOTSUP;
}

static int icm20649_sample_fetch_accel(struct device *dev)
{
	struct icm20649_data *data = dev->driver_data;

	if (icm20649_read_s16(dev, ICM20649_REG_ACCEL_XOUT_L, ICM20649_REG_ACCEL_XOUT_H, &data->accel_sample_x) < 0) {
		SYS_LOG_DBG("failed to fetch accel x sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_ACCEL_YOUT_L, ICM20649_REG_ACCEL_YOUT_H, &data->accel_sample_y) < 0) {
		SYS_LOG_DBG("failed to fetch accel y sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_ACCEL_ZOUT_L, ICM20649_REG_ACCEL_ZOUT_H, &data->accel_sample_z) < 0) {
		SYS_LOG_DBG("failed to fetch accel z sample");
		return -EIO;
	}
	return 0;
}

static int icm20649_sample_fetch_gyro(struct device *dev)
{
	struct icm20649_data *data = dev->driver_data;

	if (icm20649_read_s16(dev, ICM20649_REG_GYRO_XOUT_L, ICM20649_REG_GYRO_XOUT_H, &data->gyro_sample_x) < 0) {
		SYS_LOG_DBG("failed to fetch gyro x sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_GYRO_YOUT_L, ICM20649_REG_GYRO_YOUT_H, &data->gyro_sample_y) < 0) {
		SYS_LOG_DBG("failed to fetch gyro y sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_GYRO_ZOUT_L, ICM20649_REG_GYRO_ZOUT_H, &data->gyro_sample_z) < 0) {
		SYS_LOG_DBG("failed to fetch gyro z sample");
		return -EIO;
	}
	return 0;
}


static int icm20649_sample_fetch_temp(struct device *dev)
{
	struct icm20649_data *data = dev->driver_data;

	if (icm20649_read_s16(dev, ICM20649_REG_TEMP_OUT_L, ICM20649_REG_TEMP_OUT_H, &data->temp_sample) < 0) {
		SYS_LOG_DBG("failed to fetch temperature");
		return -EIO;
	}
	return 0;
}

static int icm20649_sample_fetch_all(struct device *dev)
{
	u8_t spi_buf[14];

	struct icm20649_data *data = dev->driver_data;

	if (icm20649_raw_read(data, ICM20649_REG_ACCEL_XOUT_H, spi_buf, 14) < 0) {
		SYS_LOG_DBG("failed to fetch all sensor values");
		return -EIO;
	}
	data->accel_sample_x = (s16_t)((((u16_t)spi_buf[0]) << 8) | (u16_t)spi_buf[1]);
	data->accel_sample_y = (s16_t)((((u16_t)spi_buf[2]) << 8) | (u16_t)spi_buf[3]);
	data->accel_sample_z = (s16_t)(((u16_t)spi_buf[4] << 8) | (u16_t)spi_buf[5]);
	data->gyro_sample_x = (s16_t)(((u16_t)spi_buf[6] << 8) | (u16_t)spi_buf[7]);
	data->gyro_sample_y = (s16_t)(((u16_t)spi_buf[8] << 8) | (u16_t)spi_buf[9]);
	data->gyro_sample_z = (s16_t)((((u16_t)spi_buf[10]) << 8) | (u16_t)spi_buf[11]);
	data->temp_sample  = (s16_t)((((u16_t)spi_buf[12]) << 8) | (u16_t)spi_buf[13]);
	return 0;
}



static int icm20649_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_sample_fetch_accel(dev);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_sample_fetch_gyro(dev);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20649_sample_fetch_temp(dev);
		break;
	case SENSOR_CHAN_ALL:
		icm20649_sample_fetch_all(dev);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static inline void icm20649_gyro_convert(struct sensor_value *val, s16_t raw_val)
{
	/* Degrees per second, without conversion */
	val->val1 = raw_val;
}

static inline int icm20649_gyro_channel_get(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct icm20649_data *data)
{
	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		icm20649_gyro_convert(val, data->gyro_sample_x);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20649_gyro_convert(val, data->gyro_sample_y);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20649_gyro_convert(val, data->gyro_sample_z);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_gyro_convert(&val[0], data->gyro_sample_x);
		icm20649_gyro_convert(&val[1], data->gyro_sample_y);
		icm20649_gyro_convert(&val[2], data->gyro_sample_z);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}


static void icm20649_gyro_channel_get_temp(struct sensor_value *val,
					  struct icm20649_data *data)
{
	/* val = temp_sample / 256 + 25 */
	val->val1 = data->temp_sample / 256 + 25;
	val->val2 = (data->temp_sample % 256) * (1000000 / 256);
}

static inline void icm20649_accel_convert(struct sensor_value *val, s16_t raw_val)
{
	/* exposed as gravity in G's*/
	val->val1 = raw_val;

}


static inline int icm20649_accel_channel_get(enum sensor_channel chan,
					    struct sensor_value *val,
					    struct icm20649_data *data)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		icm20649_accel_convert(val, data->accel_sample_x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20649_accel_convert(val, data->accel_sample_y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20649_accel_convert(val, data->accel_sample_z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_accel_convert(&val[0], data->accel_sample_x);
		icm20649_accel_convert(&val[1], data->accel_sample_y);
		icm20649_accel_convert(&val[2], data->accel_sample_z);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}


static int icm20649_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct icm20649_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_accel_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_gyro_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20649_gyro_channel_get_temp(val, data);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}


static const struct sensor_driver_api icm20649_api_funcs = {
	.attr_set = icm20649_attr_set,
#if CONFIG_ICM20649_TRIGGER
	.trigger_set = icm20649_trigger_set,
#endif
	.sample_fetch = icm20649_sample_fetch,
	.channel_get = icm20649_channel_get,
};

static int icm20649_init_chip(struct device *dev) {
	u8_t chip_id;

	if(icm20649_set_user_bank(dev, 0) < 0) {
		SYS_LOG_DBG("failed to set user bank to 0");
		return -EIO;
	}

	if(icm20649_get_who_am_i(dev, &chip_id) < 0) {
		SYS_LOG_DBG("failed reading chip id");
		return -EIO;
	}

	if(chip_id != ICM20649_VAL_WHO_AM_I) {
		SYS_LOG_DBG("invalid chip id 0x%x", chip_id);
		return -EIO;
	}
	
	if(icm20649_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset device");
		return -EIO;
	}

	k_sleep(K_MSEC(10));

	if(icm20649_auto_clock(dev) < 0) {
		SYS_LOG_DBG("failed to auto clock");
		return -EIO;
	}

	if(icm20649_wakeup(dev) < 0) {
		SYS_LOG_DBG("failed to power on device");
		return -EIO;
	}

	if(icm20649_user_config(dev) < 0) {
		SYS_LOG_DBG("failed to config device for usage");
		return -EIO;
	}

	k_sleep(K_MSEC(10));

    if(icm20649_lp_enable(dev) < 0) {
        SYS_LOG_DBG("failed to turn on low power mode for analog circuitry");
        return -EIO;
    }

	k_sleep(K_MSEC(10));


    if(icm20649_accel_gyro_enable(dev) < 0 ) {
        SYS_LOG_DBG("failed to enable accelerometer and gyroscope");
        return -EIO;
    }

	if(icm20649_set_odr_align(dev, 1) < 0) {
		SYS_LOG_DBG("failed to set odr align");
		return -EIO;
	}

	if(icm20649_set_user_bank(dev, 2) < 0) {
		SYS_LOG_DBG("failed to set user bank to 2");
		return -EIO;
	}

	if(icm20649_set_accel_filter_choice(dev, ICM20649_DEFAULT_ACCEL_LPF_ENABLE) < 0) {
		SYS_LOG_DBG("failed to set accelerometer low-pass filter settings");
		return -EIO;
	}

	if(icm20649_set_accel_sample_rate_div(dev, ICM20649_DEFAULT_ACCEL_SAMPLE_RATE_DIV) < 0) {
		SYS_LOG_DBG("failed to set accelerometer sample rate divider");
		return -EIO;
	}

	/*
	if(icm20649_set_accel_decimator(dev, ICM20649_DEFAULT_ACCEL_DECIMATOR) < 0) {
		SYS_LOG_DBG("failed to set accelerometer sample averaging decimator");
		return -EIO;
	}
	*/


	if(icm20649_set_accel_lpf(dev, ICM20649_DEFAULT_ACCEL_LPF_CFG) < 0) {
		SYS_LOG_DBG("failed to set accelerometer low-pass filter settings");
		return -EIO;
	}
	
	if(icm20649_set_accel_fs_sel(dev, ICM20649_DEFAULT_ACCEL_FULLSCALE) < 0) {
		SYS_LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	if(icm20649_set_gyro_sample_rate_div(dev, ICM20649_DEFAULT_GYRO_SAMPLE_RATE_DIV) < 0) {
		SYS_LOG_DBG("failed to set gyro sample rate divider");
		return -EIO;
	}

	/*
	if(icm20649_set_gyro_decimator(dev, ICM20649_DEFAULT_GYRO_DECIMATOR) < 0) {
		SYS_LOG_DBG("failed to set accelerometer sample averaging decimator");
		return -EIO;
	}
	*/
	
	if(icm20649_set_gyro_filter_choice(dev, ICM20649_DEFAULT_GYRO_LPF_ENABLE) < 0) {
		SYS_LOG_DBG("failed to set gyroscope low-pass filter settings");
		return -EIO;
	}

	if(icm20649_set_gyro_lpf(dev, ICM20649_DEFAULT_GYRO_LPF_CFG) < 0) {
		SYS_LOG_DBG("failed to set gyroscope low-pass filter settings");
		return -EIO;
	}

	if(icm20649_set_gyro_fs_sel(dev, ICM20649_DEFAULT_GYRO_FULLSCALE) < 0) {
		SYS_LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	if(icm20649_set_user_bank(dev, 0) < 0) {
		SYS_LOG_DBG("failed to set user bank to 2");
		return -EIO;
	}
	
	SYS_LOG_DBG("Successfully initialized ICM20649");
	return 0;
}

#ifdef CONFIG_ICM20649_TRIGGER

static inline int icm20649_int_pin_init(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_INT_PIN_CFG,
			ICM20649_MASK_INT_PIN_CFG_INT1_ACTL |
			ICM20649_MASK_INT_PIN_CFG_INT1_OPEN |
			ICM20649_MASK_INT_PIN_CFG_INT1_LATCH_EN,
			0 << ICM20649_SHIFT_INT_PIN_CFG_INT1_ACTL |
			0 << ICM20649_SHIFT_INT_PIN_CFG_INT1_OPEN | 
			0 << ICM20649_SHIFT_INT_PIN_CFG_INT1_LATCH_EN
			);
}

static inline int icm20649_fifo_watermark_int_enable(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_3,
			ICM20649_MASK_INT_ENABLE_3_FIFO_WM_EN,
			1 << ICM20649_SHIFT_INT_ENABLE_3_FIFO_WM_EN);
}

static inline int icm20649_fifo_watermark_int_disable(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_3,
			ICM20649_MASK_INT_ENABLE_3_FIFO_WM_EN,
			0 << ICM20649_SHIFT_INT_ENABLE_3_FIFO_WM_EN);
}

static inline int icm20649_fifo_overflow_int_enable(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_2,
			ICM20649_MASK_INT_ENABLE_2_FIFO_OVERFLOW_EN,
			1 << ICM20649_SHIFT_INT_ENABLE_2_FIFO_OVERFLOW_EN);
}

static inline int icm20649_raw_data_ready_int_disable(struct device *dev) {
	return icm20649_update_reg8(dev, ICM20649_REG_INT_ENABLE_1,
			ICM20649_MASK_INT_ENABLE_1_RAW_DATA_0_RDY_EN,
			0 <<  ICM20649_SHIFT_INT_ENABLE_1_RAW_DATA_0_RDY_EN);
}

int icm20649_fifo_watermark_int_status(struct device *dev, u8_t *status) {
	return icm20649_read_reg8(dev, ICM20649_REG_INT_STATUS_3, status);
}

static inline int icm20649_fifo_watermark_int_clear(struct device *dev) {
	return icm20649_write_reg8(dev, ICM20649_REG_INT_STATUS_3, 0);
}

int icm20649_fifo_overflow_int_status(struct device *dev, u8_t *status) {
	return icm20649_read_reg8(dev, ICM20649_REG_INT_STATUS_2, status);
}

static inline int icm20649_fifo_overflow_int_clear(struct device *dev) {
	return icm20649_write_reg8(dev, ICM20649_REG_INT_STATUS_2, 0);
}

static inline int icm20649_fifo_config(struct device *dev) {
	int ret = icm20649_update_reg8(dev, ICM20649_REG_FIFO_EN_2,
			ICM20649_MASK_FIFO_EN_2_ACCEL_FIFO_EN |
			ICM20649_MASK_FIFO_EN_2_GYRO_Z_FIFO_EN |
			ICM20649_MASK_FIFO_EN_2_GYRO_Y_FIFO_EN |
			ICM20649_MASK_FIFO_EN_2_GYRO_X_FIFO_EN |
			ICM20649_MASK_FIFO_EN_2_TEMP_FIFO_EN,
			1 << ICM20649_SHIFT_FIFO_EN_2_ACCEL_FIFO_EN |
			1 << ICM20649_SHIFT_FIFO_EN_2_GYRO_Z_FIFO_EN |
			1 << ICM20649_SHIFT_FIFO_EN_2_GYRO_Y_FIFO_EN |
			1 << ICM20649_SHIFT_FIFO_EN_2_GYRO_X_FIFO_EN |
			0 << ICM20649_SHIFT_FIFO_EN_2_TEMP_FIFO_EN
			);
	ret |= icm20649_update_reg8(dev, ICM20649_REG_FIFO_EN_1,
			ICM20649_MASK_FIFO_EN_SLV_3_FIFO_EN |
			ICM20649_MASK_FIFO_EN_SLV_2_FIFO_EN |
			ICM20649_MASK_FIFO_EN_SLV_1_FIFO_EN |
			ICM20649_MASK_FIFO_EN_SLV_0_FIFO_EN,
			0 << ICM20649_SHIFT_FIFO_EN_SLV_3_FIFO_EN |
			0 << ICM20649_SHIFT_FIFO_EN_SLV_2_FIFO_EN |
			0 << ICM20649_SHIFT_FIFO_EN_SLV_1_FIFO_EN |
			0 << ICM20649_SHIFT_FIFO_EN_SLV_0_FIFO_EN
			);
	ret |= icm20649_update_reg8(dev, ICM20649_REG_FIFO_MODE,
			ICM20649_MASK_FIFO_MODE,
			ICM20649_FIFO_MODE_STREAM);
	return ret;
}

static inline int icm20649_fifo_reset(struct device *dev) {
	u16_t len = ICM20649_FIFO_SIZE;
	u8_t tries = 0;
	int ret = 0;
	while(len != 0 && tries < 6) {
		ret = icm20649_update_reg8(dev, ICM20649_REG_USER_CTRL,
				ICM20649_MASK_USER_CTRL_DMP_EN |
				ICM20649_MASK_USER_CTRL_FIFO_EN,
				0 << ICM20649_SHIFT_USER_CTRL_DMP_EN |
				0 << ICM20649_SHIFT_USER_CTRL_FIFO_EN
				);
		ret |= icm20649_update_reg8(dev, ICM20649_REG_FIFO_RST,
				ICM20649_MASK_FIFO_RST,
				1 << ICM20649_SHIFT_FIFO_RST);
		ret |= icm20649_fifo_count(dev, &len);
		if(ret) {
			return ret;
		}
	}
	ret |= icm20649_update_reg8(dev, ICM20649_REG_FIFO_RST,
			ICM20649_MASK_FIFO_RST,
			0 << ICM20649_SHIFT_FIFO_RST);
	ret |= icm20649_update_reg8(dev, ICM20649_REG_USER_CTRL,
			ICM20649_MASK_USER_CTRL_DMP_EN |
			ICM20649_MASK_USER_CTRL_FIFO_EN,
			0 << ICM20649_SHIFT_USER_CTRL_DMP_EN |
			1 << ICM20649_SHIFT_USER_CTRL_FIFO_EN
			);

	return ret;
}

int icm20649_fifo_count(struct device *dev, u16_t *cnt) {
	struct icm20649_data *data = dev->driver_data;
	u8_t read_buf[2];
    int ret = icm20649_raw_read(data, ICM20649_REG_FIFO_COUNTH, read_buf, 2);
	*cnt = ((u16_t)read_buf[0] << 8) + (u16_t)read_buf[1];
	return ret;
}

static inline int icm20649_fifo_disable(struct device *dev) {
	if(icm20649_fifo_watermark_int_disable(dev) < 0) {
		SYS_LOG_DBG("failed to enable FIFO watermark interrupt");
		return -EIO;
	}
	if(icm20649_fifo_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset FIFO");
		return -EIO;
	}
	return 0;
}


static inline int icm20649_fifo_enable(struct device *dev) {
	if(icm20649_int_pin_init(dev) < 0) {
		SYS_LOG_DBG("failed to configure interrupt pin");
		return -EIO;
	}
	if(icm20649_raw_data_ready_int_disable(dev) < 0) {
		SYS_LOG_DBG("failed to disable raw data interrupt");
		return -EIO;
	}
	if(icm20649_fifo_watermark_int_enable(dev) < 0) {
		SYS_LOG_DBG("failed to enable FIFO watermark interrupt");
		return -EIO;
	}

	/*
	if(icm20649_fifo_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset FIFO");
		return -EIO;
	}
	*/
	/*
	if(icm20649_fifo_overflow_int_enable(dev) < 0) {
		SYS_LOG_DBG("failed to enable FIFO overflow interrupt");
		return -EIO;
	}
	*/
	if(icm20649_fifo_config(dev) < 0) {
		SYS_LOG_DBG("failed to configure FIFO");
		return -EIO;
	}
	if(icm20649_fifo_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset FIFO");
		return -EIO;
	}
	return 0;
}

int icm20649_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct icm20649_data *drv_data = dev->driver_data;

	if(handler == NULL) {
		SYS_LOG_DBG("Clearing trigger");

		gpio_pin_disable_callback(drv_data->gpio, CONFIG_ICM20649_GPIO_PIN_NUM);

		/* enable fifo and watermark interrupt */
		if(icm20649_fifo_disable(dev) < 0) {
			SYS_LOG_ERR("Could not disable fifo watermark interrupt.");
			return -EIO;
		}

	} else {
		SYS_LOG_DBG("Setting trigger");
		
		__ASSERT_NO_MSG(trig->type == SENSOR_TRIG_DATA_READY);

		gpio_pin_disable_callback(drv_data->gpio, CONFIG_ICM20649_GPIO_PIN_NUM);

		drv_data->data_ready_handler = handler;
		drv_data->data_ready_trigger = *trig;

		gpio_pin_enable_callback(drv_data->gpio, CONFIG_ICM20649_GPIO_PIN_NUM);

		/* enable fifo and watermark interrupt */
		if(icm20649_fifo_enable(dev) < 0) {
			SYS_LOG_ERR("Could not enable fifo watermark interrupt.");
			return -EIO;
		}

	}

	return 0;
}

static void icm20649_gpio_callback(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	struct icm20649_data *drv_data =
		CONTAINER_OF(cb, struct icm20649_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_disable_callback(dev, CONFIG_ICM20649_GPIO_PIN_NUM);

#if defined(CONFIG_ICM20649_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_ICM20649_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}


static void icm20649_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct icm20649_data *drv_data = dev->driver_data;

    SYS_LOG_DBG("Interrupt triggered");

	// read data from fifo as an array of s16_t's up to 512bytes worth
    // pass data to callback given at setup time
	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	//icm20649_fifo_watermark_int_clear(dev);
	//icm20649_fifo_overflow_int_clear(dev);
	gpio_pin_enable_callback(drv_data->gpio, CONFIG_ICM20649_GPIO_PIN_NUM);
}

#ifdef CONFIG_ICM20649_TRIGGER_OWN_THREAD
static void icm20649_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct icm20649_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		icm20649_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_ICM20649_TRIGGER_GLOBAL_THREAD
static void icm20649_work_cb(struct k_work *work)
{
	struct icm20649_data *drv_data =
		CONTAINER_OF(work, struct icm20649_data, work);

	icm20649_thread_cb(drv_data->dev);
}
#endif

int icm20649_init_interrupt(struct device *dev)
{
    SYS_LOG_DBG("Enabling interrupt");
	struct icm20649_data *drv_data = dev->driver_data;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(CONFIG_ICM20649_GPIO_DEV_NAME);
	if (drv_data->gpio == NULL) {
		SYS_LOG_ERR("Cannot get pointer to %s device.",
			    CONFIG_ICM20649_GPIO_DEV_NAME);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->gpio, CONFIG_ICM20649_GPIO_PIN_NUM,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_HIGH | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&drv_data->gpio_cb,
			   icm20649_gpio_callback,
			   BIT(CONFIG_ICM20649_GPIO_PIN_NUM));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		SYS_LOG_ERR("Could not set gpio callback.");
		return -EIO;
	}

#if defined(CONFIG_ICM20649_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ICM20649_THREAD_STACK_SIZE,
			(k_thread_entry_t)icm20649_thread, POINTER_TO_INT(dev),
			0, NULL, K_PRIO_COOP(CONFIG_ICM20649_THREAD_PRIORITY),
			0, 0);
#elif defined(CONFIG_ICM20649_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = icm20649_work_cb;
	drv_data->dev = dev;
#endif

	gpio_pin_enable_callback(drv_data->gpio, CONFIG_ICM20649_GPIO_PIN_NUM);

	return 0;
}

#endif /* ICM20649_TRIGGER */


u16_t icm20649_fifo_read(struct device *dev, u8_t *buf, u16_t len) {
	struct icm20649_data *data = dev->driver_data;
	u16_t read_len = 0;
	while(read_len < len) {
		u16_t n = min(ICM20649_MAX_SERIAL_READ, len-read_len);
		if(icm20649_raw_read(data, ICM20649_REG_FIFO_R_W, &buf[read_len], n) < 0) {
			SYS_LOG_DBG("failed to read fifo");
			return 0;
		}
		read_len += n;
	}
	return read_len;
}

/*
int icm20649_get_x_accel(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, ACCEL_XOUT_H, ACCEL_XOUT_L, x);
}

int icm20649_get_y_accel(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, ACCEL_YOUT_H, ACCEL_YOUT_L, y);
}

int icm20649_get_z_accel(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, ACCEL_ZOUT_H, ACCEL_ZOUT_L, z);
}

int icm20649_get_x_gyro(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, GYRO_XOUT_H, GYRO_XOUT_L, x);
}

int icm20649_get_y_gyro(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, GYRO_YOUT_H, GYRO_YOUT_L, y);
}

int icm20649_get_z_gyro(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, GYRO_ZOUT_H, GYRO_ZOUT_L, z);
}

int icm20649_get_temp(struct icm20649 *device, int16_t *temp) {
    return icm20649_read_i16(device, TEMP_OUT_H, TEMP_OUT_L, temp);
}

int icm20649_get_x_gyro_offset(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

int icm20649_get_y_gyro_offset(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

int icm20649_get_z_gyro_offset(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

int icm20649_set_x_gyro_offset(struct icm20649 *device, int16_t x) {
    return icm20649_write_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

int icm20649_set_y_gyro_offset(struct icm20649 *device, int16_t y) {
    return icm20649_write_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

int icm20649_set_z_gyro_offset(struct icm20649 *device, int16_t z) {
    return icm20649_write_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

int icm20649_get_x_accel_offset(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

int icm20649_get_y_accel_offset(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

int icm20649_get_z_accel_offset(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}

int icm20649_set_x_accel_offset(struct icm20649 *device, int16_t x) {
    return icm20649_write_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

int icm20649_set_y_accel_offset(struct icm20649 *device, int16_t y) {
    return icm20649_write_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

int icm20649_set_z_accel_offset(struct icm20649 *device, int16_t z) {
    return icm20649_write_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}
*/


static struct icm20649_config icm20649_config = {
	.dev_name = CONFIG_ICM20649_SPI_MASTER_DEV_NAME,
};


int icm20649_init(struct device *dev) {
	const struct icm20649_config * const config = &icm20649_config;
	struct icm20649_data *data = dev->driver_data;

	data->spi = device_get_binding(config->dev_name);
	if (!data->spi) {
		SYS_LOG_DBG("master not found: %s",
			    config->dev_name);
		return -EINVAL;
	}

#if defined(CONFIG_ICM20649_SPI_GPIO_CS)
	/* handle SPI CS thru GPIO if it is the case */
	if (IS_ENABLED(CONFIG_ICM20649_SPI_GPIO_CS)) {
		icm20649_cs_ctrl.gpio_dev = device_get_binding(
			CONFIG_ICM20649_SPI_GPIO_CS_DRV_NAME);
		if (!icm20649_cs_ctrl.gpio_dev) {
			SYS_LOG_ERR("Unable to get GPIO SPI CS device");
			return -ENODEV;
		}

		icm20649_cs_ctrl.gpio_pin = CONFIG_ICM20649_SPI_GPIO_CS_PIN;
		icm20649_cs_ctrl.delay = 0;

		icm20649_spi_conf.cs = &icm20649_cs_ctrl;

		SYS_LOG_DBG("SPI GPIO CS configured on %s:%u",
			    CONFIG_ICM20649_SPI_GPIO_CS_DRV_NAME,
			    CONFIG_ICM20649_SPI_GPIO_CS_PIN);
	}
#endif

	if (icm20649_init_chip(dev) < 0) {
		SYS_LOG_DBG("failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_ICM20649_TRIGGER
    if(icm20649_init_interrupt(dev) < 0 ) {
        SYS_LOG_DBG("failed to initialize chip");
		return -EIO;
    }
#endif

	return 0;
}


static struct icm20649_data icm20649_driver;

DEVICE_AND_API_INIT(icm20649, CONFIG_ICM20649_NAME, icm20649_init,
        &icm20649_driver, NULL, POST_KERNEL,
        CONFIG_SENSOR_INIT_PRIORITY, &icm20649_api_funcs);
