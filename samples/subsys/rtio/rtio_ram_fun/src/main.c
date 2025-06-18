/*
 * Copyright (c) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/i2c.h>

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

/*
 * Get i2c ram configuration from the devicetree i2c_ram alias. This is mandatory.
 */
#define I2C_DEV_NODE DT_ALIAS(i2c_ram)
#if !DT_NODE_HAS_STATUS_OKAY(I2C_DEV_NODE)
#error "Unsupported board: i2c_ram is not defined"
#endif

const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);

/* Address from datasheet is 0b1010xxxr where x bits are additional
 * memory address bits and r is the r/w i2c bit.
 *
 * However... the address needs to be shifted into the lower 7 bits as
 * Zephyr expects a 7bit device address and shifts this left to set the
 * i2c r/w bit.
 */
#define RAM_ADDR (0b10100010 >> 1)

I2C_IODEV_DEFINE(i2c_iodev, DT_ALIAS(i2c_ram), RAM_ADDR);
RTIO_DEFINE(r_btn, 32, 0);

const static uint16_t addr[4] = {
	0x0010,
	0x0020,
	0x0030,
	0x0040,
};

static uint32_t iteration;
static uint32_t data[4][2];
static uint64_t press_ts;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	press_ts = k_cycle_get_32();

	struct rtio_sqe *wraddr_sqe, *wrdata_sqe;

	/* Write several addresses with a string */

	for (int i = 0; i < 4; i++) {
		data[i][0] = iteration;
		data[i][1] = press_ts;

		wraddr_sqe = rtio_sqe_acquire(&r_btn);
		wrdata_sqe = rtio_sqe_acquire(&r_btn);

		if (wraddr_sqe == NULL || wrdata_sqe == NULL) {
			printk("out of submissions!\n");
		}

		rtio_sqe_prep_write(wraddr_sqe, &i2c_iodev, 0, (uint8_t *)&addr[i], 2, NULL);
		wraddr_sqe->flags |= RTIO_SQE_TRANSACTION | RTIO_SQE_NO_RESPONSE;
		rtio_sqe_prep_write(wrdata_sqe, &i2c_iodev, 0, (const uint8_t *)&data[i], 8, NULL);
		wrdata_sqe->iodev_flags |= RTIO_IODEV_I2C_STOP;
		wrdata_sqe->flags |= RTIO_SQE_NO_RESPONSE;
		iteration += 1;

		rtio_submit(&r_btn, 0);
	}
}

RTIO_DEFINE(r_main, 8, 8);

int main(void)
{
	int ret;
	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

	if (!device_is_ready(i2c_dev)) {
		printk("Error: i2c device %s is not ready\n", i2c_dev->name);
		return 0;
	}

	if ((ret = i2c_configure(i2c_dev, i2c_cfg)) != 0) {
		printk("Error: i2c device %s failed to configure\n", i2c_dev->name);
		return 0;
	}

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return 0;
	}

	if ((ret = gpio_pin_configure_dt(&button, GPIO_INPUT)) != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name,
		       button.pin);
		return 0;
	}

	if ((ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE)) != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
		       button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	struct rtio_sqe *wraddr_sqe, *rddata_sqe;
	struct rtio_cqe *cqe;

	uint32_t rd_data[4][2];
	while (true) {
		k_sleep(K_MSEC(1000));

		printk("reading data...\n");

		for (int i = 0; i < 4; i++) {
			wraddr_sqe = rtio_sqe_acquire(&r_main);
			rddata_sqe = rtio_sqe_acquire(&r_main);

			rtio_sqe_prep_tiny_write(wraddr_sqe, &i2c_iodev, 0, (uint8_t *)&addr[i],
						 sizeof(addr[i]), NULL);
			wraddr_sqe->flags |= RTIO_SQE_TRANSACTION;
			rtio_sqe_prep_read(rddata_sqe, &i2c_iodev, 0, (uint8_t *)&rd_data[i], 8,
					   NULL);
			rddata_sqe->iodev_flags |= RTIO_IODEV_I2C_STOP;
		}

		printk("submitting...\n");
		rtio_submit(&r_btn, 4);

		printk("checking results\n");
		for (int i = 0; i < 4; i++) {
			cqe = rtio_cqe_consume(&r_main);
			if (cqe->result != 0) {
				printk("failed to read data from ram, result %d address %x\n",
				       cqe->result, addr[i]);
			} else {
				printk("Data at addr %x iteration %d cycle %d\n", addr[i],
				       rd_data[i][0], rd_data[i][1]);
			}
			rtio_cqe_release(&r_main, cqe);
		}
	}
}
