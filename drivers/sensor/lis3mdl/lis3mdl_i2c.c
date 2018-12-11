#include <i2c.h>
#include "lis3mdl.h"

static u16_t lis3mdl_i2c_slave_addr = CONFIG_LIS3MDL_I2C_ADDR;

static int lis3mdl_i2c_read_data(struct lis3mdl_data *data, u8_t reg_addr,
                                 u8_t *value, u8_t len)
{
	return i2c_burst_read(data->comm_master, lis3mdl_i2c_slave_addr,
                        reg_addr, value, len);
}

static int lis3mdl_i2c_write_data(struct lis3mdl_data *data, u8_t reg_addr,
                                  u8_t *value, u8_t len)
{
	return i2c_burst_write(data->comm_master, lis3mdl_i2c_slave_addr,
                         reg_addr, value, len);
}

static int lis3mdl_i2c_read_reg(struct lis3mdl_data *data, u8_t reg_addr,
                                u8_t *value)
{
	return i2c_reg_read_byte(data->comm_master, lis3mdl_i2c_slave_addr,
                           reg_addr, value);
}

static struct lis3mdl_transfer_function lis3mdl_i2c_tf = {
	.read_data = lis3mdl_i2c_read_data,
	.write_data = lis3mdl_i2c_write_data,
	.read_reg = lis3mdl_i2c_read_reg,
};

int lis3mdl_i2c_init(struct lis3mdl_data *drv_data) {
	int res = 0;

#ifdef CONFIG_LIS3MDL_I2C_GPIO_CS
	struct device *cs_gpio = device_get_binding(CONFIG_LIS3MDL_I2C_GPIO_CS_DRV_NAME);
	res = gpio_pin_configure(cs_gpio, CONFIG_LIS3MDL_I2C_GPIO_CS_PIN_NUM, GPIO_DIR_OUT);
	if(res != 0) {
		SYS_LOG_ERR("Could not configure CS pin for I2C usage, %d cause res %d", CONFIG_LIS3MDL_I2C_GPIO_CS_PIN_NUM, res);
		return -EINVAL;
	}
	res = gpio_pin_write(cs_gpio, CONFIG_LIS3MDL_I2C_GPIO_CS_PIN_NUM, 1);
	if(res != 0) {
		SYS_LOG_ERR("Could not write CS pin for I2C usage, %d cause res %d", CONFIG_LIS3MDL_I2C_GPIO_CS_PIN_NUM, res);
		return -EINVAL;
	}
#endif

#ifdef CONFIG_LIS3MDL_I2C_GPIO_SA1
	struct device *sa1_gpio = device_get_binding(CONFIG_LIS3MDL_I2C_GPIO_SA1_DRV_NAME);
	res = gpio_pin_configure(sa1_gpio, CONFIG_LIS3MDL_GPIO_SA1_PIN_NUM, GPIO_DIR_OUT | GPIO_PUD_PULL_DOWN);
	if(res != 0) {
		SYS_LOG_ERR("Could not configure SA1 pin, %d cause res %d", CONFIG_LIS3MDL_I2C_GPIO_SA1_PIN_NUM, res);
		return -EINVAL;
	}
#if CONFIG_LIS3MDL_I2C_ADDR == "0x1E"
#define LIS3MDL_SA1_VALUE 1
#else
#define LIS3MDL_SA1_VALUE 0
#endif
	res = gpio_pin_write(gpio_dev, CONFIG_LIS3MDL_GPIO_SA1_PIN_NUM, LIS3MDL_SA1_VALUE);
	if(res != 0) {
		SYS_LOG_ERR("Could not write SA1 pin, %d cause res %d", CONFIG_LIS3MDL_GPIO_SA1_PIN_NUM, res);
		return -EINVAL;
	}
#endif

  drv_data->hw_tf = &lis3mdl_i2c_tf;

  return res;
}
