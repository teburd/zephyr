/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/rtio/rtio_spsc.h>
#include <zephyr/sys/__assert.h>

const struct rtio_iodev_api i2c_iodev_api = {
	.submit = i2c_iodev_submit,
};

struct rtio_sqe *i2c_rtio_copy(struct rtio *r,
			       struct rtio_iodev *iodev,
			       const struct i2c_msg *msgs,
			       uint8_t num_msgs)
{
	__ASSERT(num_msgs > 0, "Expecting at least one message to copy");

	struct rtio_sqe *sqe = NULL;

	for (uint8_t i = 0; i < num_msgs; i++) {
		sqe = rtio_sqe_acquire(r);

		if (sqe == NULL) {
			rtio_sqe_drop_all(r);
			return NULL;
		}

		if (msgs[i].flags & I2C_MSG_READ) {
			rtio_sqe_prep_read(sqe, iodev, RTIO_PRIO_NORM,
					   msgs[i].buf, msgs[i].len, NULL);
		} else {
			rtio_sqe_prep_write(sqe, iodev, RTIO_PRIO_NORM,
					    msgs[i].buf, msgs[i].len, NULL);
		}
		sqe->flags |= RTIO_SQE_TRANSACTION;
		sqe->iodev_flags = ((msgs[i].flags & I2C_MSG_STOP) ? RTIO_IODEV_I2C_STOP : 0) |
			((msgs[i].flags & I2C_MSG_RESTART) ? RTIO_IODEV_I2C_RESTART : 0) |
			((msgs[i].flags & I2C_MSG_ADDR_10_BITS) ? RTIO_IODEV_I2C_10_BITS : 0);
	}

	sqe->flags &= ~RTIO_SQE_TRANSACTION;

	return sqe;
}


static int i2c_rtio_transfer(const struct i2c_rtio *ctx, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr)
	struct i2c_rtio *const dev_data = ctx;
	struct rtio_iodev *iodev = &dev_data->iodev;
	struct rtio *const r = dev_data->r;
	struct rtio_sqe *sqe = NULL;
	struct rtio_cqe *cqe = NULL;
	int res = 0;

	k_sem_take(&dev_data->r_lock, K_FOREVER);

	dev_data->dt_spec.addr = addr;

	sqe = i2c_rtio_copy(r, iodev, msgs, num_msgs);
	if (sqe == NULL) {
		LOG_ERR("Not enough submission queue entries");
		res = -ENOMEM;
		goto out;
	}

	sqe->flags &= ~RTIO_SQE_TRANSACTION;

	rtio_submit(r, 1);

	cqe = rtio_cqe_consume(r);
	__ASSERT(cqe != NULL, "Expected valid completion");
	while (cqe != NULL) {
		res = cqe->result;
		cqe = rtio_cqe_consume(r);
	}
	rtio_cqe_release_all(r);

out:
	k_sem_give(&dev_data->r_lock);
	return res;

}

static int i2c_rtio_recovery(const struct i2c_rtio *ctx)
{
	struct i2c_rtio *const ctx = dev->data;
	struct rtio_iodev *iodev = &ctx->iodev;
	struct rtio *const r = ctx->r;
	struct rtio_sqe *sqe = NULL;
	struct rtio_cqe *cqe = NULL;
	int res = 0;

	k_sem_take(&dev_data->r_lock, K_FOREVER);

	sqe = rtio_sqe_acquire(r);
	if (sqe == NULL) {
		LOG_ERR("Not enough submission queue entries");
		res = -ENOMEM;
		goto out;
	}

	sqe->op = RTIO_SQE_OP_I2C_RECOVERY;
	sqe->iodev = iodev;

	rtio_submit(r, 1);

	cqe = rtio_cqe_consume(r);
	res = cqe->result;
	__ASSERT(cqe != NULL, "Expected valid completion");
	rtio_cqe_release_all(r);

out:
	k_sem_give(&ctx->r_lock);
	return res;
}

