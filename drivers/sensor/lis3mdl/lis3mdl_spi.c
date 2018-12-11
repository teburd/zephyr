
#include <spi.h>
#include "lis3mdl.h"

#define LIS3MDL_SPI_READ		(1 << 7)
#define LIS3MDL_SPI_MS			(1 << 6)

#if defined(CONFIG_LIS3MDL_SPI_GPIO_CS)
static struct spi_cs_control lis3mdl_cs_ctrl;
#endif

#define SPI_CS NULL


static struct spi_config lis3mdl_spi_conf = {
	.frequency = CONFIG_LIS3MDL_SPI_BUS_FREQ,
	.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
		      SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE),
	.slave     = CONFIG_LIS3MDL_SPI_SELECT_SLAVE,
	.cs        = SPI_CS,
};


static int lis3mdl_raw_read(struct lis3mdl_data *data, u8_t reg_addr,
			    u8_t *value, u8_t len)
{
	struct spi_config *spi_cfg = &lis3mdl_spi_conf;
	u8_t buffer_tx[2] = { reg_addr | LIS3MDL_SPI_READ | LIS3MDL_SPI_MS, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
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


	if (len > 64) {
		return -EIO;
	}

	if (spi_transceive(data->comm_master, spi_cfg, &tx, &rx)) {
		return -EIO;
	}
	return 0;
}

static int lis3mdl_raw_write(struct lis3mdl_data *data, u8_t reg_addr,
			     u8_t *value, u8_t len)
{
	struct spi_config *spi_cfg = &lis3mdl_spi_conf;
	u8_t buffer_tx[1] = { (reg_addr & ~LIS3MDL_SPI_READ) | LIS3MDL_SPI_MS };
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

	if (spi_write(data->comm_master, spi_cfg, &tx)) {
		return -EIO;
	}
	return 0;
}

static int lis3mdl_spi_read_data(struct lis3mdl_data *data, u8_t reg_addr,
				 u8_t *value, u8_t len)
{
	return lis3mdl_raw_read(data, reg_addr, value, len);
}

static int lis3mdl_spi_write_data(struct lis3mdl_data *data, u8_t reg_addr,
				  u8_t *value, u8_t len)
{
	return lis3mdl_raw_write(data, reg_addr, value, len);
}

static int lis3mdl_spi_read_reg(struct lis3mdl_data *data, u8_t reg_addr,
                                u8_t *value)
{
	return lis3mdl_raw_read(data, reg_addr, value, 1);
}



static struct lis3mdl_transfer_function lis3mdl_spi_tf = {
 .read_data = lis3mdl_spi_read_data,
 .write_data = lis3mdl_spi_write_data,
 .read_reg = lis3mdl_spi_read_reg,
};

int lis3mdl_spi_init(struct lis3mdl_data *drv_data) {
	int res = 0;

#if defined(CONFIG_LIS3MDL_SPI_GPIO_CS)
	/* handle SPI CS thru GPIO if it is the case */
	if (IS_ENABLED(CONFIG_LIS3MDL_SPI_GPIO_CS)) {
		lis3mdl_cs_ctrl.gpio_dev = device_get_binding(
			CONFIG_LIS3MDL_SPI_GPIO_CS_DRV_NAME);
		if (!lis3mdl_cs_ctrl.gpio_dev) {
			SYS_LOG_ERR("Unable to get GPIO SPI CS device");
			return -ENODEV;
		}

		lis3mdl_cs_ctrl.gpio_pin = CONFIG_LIS3MDL_SPI_GPIO_CS_PIN;
		lis3mdl_cs_ctrl.delay = 0;

		lis3mdl_spi_conf.cs = &lis3mdl_cs_ctrl;

		SYS_LOG_DBG("SPI GPIO CS configured on %s:%u",
			    CONFIG_LIS3MDL_SPI_GPIO_CS_DRV_NAME,
			    CONFIG_LIS3MDL_SPI_GPIO_CS_PIN);
	}
#endif


	drv_data->hw_tf = &lis3mdl_spi_tf;

	return res;
}
