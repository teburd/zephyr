.. _rtio_api:

RTIO
####

.. contents::
 :local:
 :depth: 2

The RTIO section covers the Real-Time I/O API, executor, and common usage
patterns with peripheral devices. RTIO attempts to provide a common API that
enables the usage of DMA or DMA-like transfers while allowing code to work in a
blocking or non-blocking way with Zephyr devices.

RTIO takes a lot of inspiration from Linux's io_uring in its operations and API.

A quick sales pitch on why RTIO works well in many scenarios:

1. API is DMA and ISR friendly.
2. Minimal copying
3. No callbacks or potentially unexpected calling context.
4. Blocking or Non-Blocking Operation with a unified API, rtio_submit is the
   sole entry point for the application to inform execution of the queue.

When to Use
***********

It's important to understand when DMA like transfers are useful and when they
are not. It's a poor idea to assume that something made for high throughput will
work for you. There is a computational, memory, and latency cost to setup the
description of transfers.

Polling at 1Hz an air sensor will almost certainly result in a net negative
result compared to ad-hoc sensor (i2c/spi) requests to get the sample.

Continuous transfers, driven by timer or interrupt, of data from a peripherals
on board FIFO over I2C, I3C, SPI, MIPI, I2S, etc... maybe, but not always!

Why a new API
*************

In Zephyr there are device APIs and then there is the DMA API. They are entirely
separate entities and there's no connection to them from an application
perspective. RTIO is an attempt to bring the two together.

DMA controllers have great features that go under utilized. RTIO attempts to
expose some of those features in a hardware agnostic way. It does not
attempt to be an end all be all DMA API. Instead favoring a certain usage
pattern, a queue of block transfers between peripherals and memory. Arguably
the most common use of DMA.

Lastly even if an application were to use the DMA API today it would not be
usable in a portable manner. The intersection of functionality support among
DMA drivers and hardware is limited. Effectively one shot transfers
which is exactly what the device APIs support in _async calls already. RTIO
providing a specific usage pattern enables software fills when the hardware support is
limited to single shot transfers while still taking advantage of hardware support
for transfer lists, bus arbitration, and more when available.

Examples
********

Examples speak loudly about the intended uses and goals of an API. So several key
examples are presented below.

Chained Blocking Requests
=========================

.. code-block:: C

	RTIO_I2C_DEVIO(i2c_dev, I2C_DT_SPEC_INST(n));
	RTIO_DEFINE(ez_io, 4, 4);
	static uint8_t bufs[2][32];

	int do_some_io(void)
	{
		struct rtio_sqe *read_sqe = rtio_fifo_acquire(ez_io.sq);
		rtio_sqe_prep_read(read_sqe, i2c_dev, RTIO_PRIO_LOW, bufs[0], 32);
		read_sqe->flags = RTIO_SQE_CHAINED; /* the next item in the queue will wait on this one */
		rtio_fifo_produce(ez_io.sq);

		struct rtio_sqe *write_sqe = rtio_fifo_acquire(ez_io.sq);
		rtio_sqe_prep_write(write_sqe, i2c_dev, RTIO_PRIO_LOW, bufs[1], 32);
	    rtio_fifo_produce(ez_io.sq);

		rtio_submit(rtio_inplace_executor, &ez_io, 2);
		struct rtio_cqe *read_cqe = rtio_fifo_consume(ez_io.cq);
		struct rtio_cqe *write_cqe = rtio_fifo_consume(ez_io.cq);
		if(read_cqe->result < 0) {
			LOG_ERR("read failed!");
		}
		if(write_cqe->result < 0) {
			LOG_ERR("write failed!");
		}
		rtio_fifo_release(ez_io.cq);
	}

Non blocking device to device
=============================

.. code-block:: C

	RTIO_I2C_DEVIO(i2c_dev, I2C_DT_SPEC_INST(n));
	RTIO_SPI_DEVIO(spi_dev, SPI_DT_SPEC_INST(m));

	RTIO_DEFINE(ez_io, 4, 4);
	static uint8_t buf[32];

	int do_some_io(void)
	{
		uint32_t read, write;
		struct rtio_sqe *read_sqe = rtio_fifo_acquire(ez_io.sq);
		rtio_sqe_prep_read(read_sqe, i2c_dev, RTIO_PRIO_LOW, buf, 32);
		read_sqe->flags = RTIO_SQE_CHAINED; /* the next item in the queue will wait on this one */
		rtio_fifo_produce(ez_io.sq);

		/* Safe to do as the chained operation *ensures* that if one fails all subsequent ops fail */
		struct rtio_sqe *write_sqe = rtio_fifo_acquire(ez_io.sq);
		rtio_sqe_prep_write(write_sqe, spi_dev, RTIO_PRIO_LOW, buf, 32);
		rtio_fifo_produce(ez_io.sq);

		/* call will return immediately without blocking if possible */
		rtio_submit(rtio_inplace_executor, &ez_io, 0);

		/* These calls might return NULL if the operations have not yet completed! */
		for (int i = 0; i < 2; i++) {
			struct rtio_cqe *cqe = rtio_fifo_consume(ez_io.cq);
			while(cqe == NULL) {
				cqe = rtio_fifo_consume(ez_io.cq);
				k_yield();
			}
			if(cqe->userdata == &read && cqe->result < 0) {
				LOG_ERR("read from i2c failed!");
			}
			if(cqe->userdata == &write && cqe->result < 0) {
				LOG_ERR("write to spi failed!");
			}
			/* Must release the completion queue event after consume */
			rtio_fifo_release(ez_io.cq);
		}
	}

Nested iodevs for Devices on Buses (Sensors), Theoretical
=========================================================

Consider a device like a sensor or flash sitting on a SPI bus.

Its useful to consider that the sensor driver can use RTIO to do I/O on the SPI
bus, while also being an RTIO device itself. The sensor iodev can set aside a
small portion of the buffer in front or in back to store some metadata describing
the format of the data. This metadata could then be used in creating a sensor
readings iterator which lazily lets you map over each reading, doing
calculations such as FIR/IIR filtering, or perhaps translating the readings into
other numerical formats with useful measurement units such as SI. RTIO is a
common movement API and allows for such uses while not deciding the mechanism.

This same sort of setup could be done for other data streams such as audio or
video.

.. code-block:: C

	/* Note that the sensor device itself can use RTIO to get data over I2C/SPI
	 * potentially with DMA, but we don't need to worry about that here
	 * All we need to know is the device tree node_id and that it can be an iodev
	 */
	RTIO_SENSOR_DEVIO(sensor_dev, DEVICE_DT_GET(DT_NODE(super6axis));

	RTIO_DEFINE(ez_io, 4, 4);


	/* The sensor driver decides the minimum buffer size for us, we decide how
	 * many bufs. This could be a typical multiple of a fifo packet the sensor
	 * produces, ICM42688 for example produces a FIFO packet of 20 bytes in
	 * 20bit mode at 32KHz so perhaps we'd like to get 4 buffers of 4ms of data
	 * each in this setup to process on. and its already been defined here for us.
	 */
	#include <sensors/icm42688_p.h>
	static uint8_t bufs[4][ICM42688_RTIO_BUF_SIZE];

	int do_some_sensors(void) {
		/* Obtain a dmac executor from the DMA device */
	    struct device *dma = DEVICE_DT_GET(DT_NODE(dma0));
	    const struct rtio_executor *rtio_dma_exec =
				dma_rtio_executor(dma);

		/* Mostly we want to feed the sensor driver enough buffers to fill while
		 * we wait! Small enough to process quickly with low latency, big enough
		 * to not spend all the time setting transfers up. No need to chain
		 * here, and no need to do any async stuff on our end.
		 *
		 * It's assumed here that the sensor has been configured already
		 * and each FIFO watermark interrupt that occurs it attempts
		 * to pull from the queue, fill the buffer with a small metadata
		 * offset using its own rtio request to the SPI bus using DMA.
		 */
		for(int i = 0; i < 4; i++) {
			struct rtio_sqe *read_sqe = rtio_fifo_acquire(ez_io.sq);

			rtio_sqe_prep_read(read_sqe, sensor_dev, RTIO_PRIO_HIGH, bufs[i], ICM42688_RTIO_BUF_SIZE);
		    rtio_fifo_produce(ez_io.sq);
		}
		struct device *sensor = DEVICE_DT_GET(DT_NODE(super6axis));
		struct sensor_reader reader;
		struct sensor_channels channels[4] = {
			SENSOR_TIMESTAMP_CHANNEL,
			SENSOR_CHANNEL(int16_t, SENSOR_ACC_X, 0, SENSOR_RAW),
			SENSOR_CHANNEL(int16_t SENSOR_ACC_Y, 0, SENSOR_RAW),
			SENSOR_CHANNEL(int16_t, SENSOR_ACC_Z, 0, SENSOR_RAW),
		};
		while (true) {
			/* call will wait for one completion event */
			rtio_submit(rtio_dma_exec, ez_io, 1);
			struct rtio_cqe *cqe = rtio_fifo_consume(ez_io.cq);
			if(cqe->result < 0) {
				LOG_ERR("read failed!");
		        goto next;
			}

			/* Bytes read into the buffer */
			int32_t bytes_read = cqe->result;

			/* Retrieve soon to be reusable buffer pointer from completion */
			uint8_t *buf = cqe->userdata;


			/* Get an iterator (reader) that obtains sensor readings in integer
			 * form, 20 bit signed values as signed 32 bit values
			 */
			res = sensor_reader(sensor, buf, cqe->result, &reader, channels,
							    sizeof(channels));
			__ASSERT(res == 0);
			while(sensor_reader_next(&reader)) {
				printf("time(cycle): %d, acc (x,y,z): (%d, %d, %d)\n",
				channels[0].value.u32, channels[1].value.i16,
				channels[2].value.i32, channels[3].value.i16);
			}

	next:
			/* Release completion queue event */
			rtio_fifo_release(ez_io.cq);

			/* resubmit a read request with the newly freed buffer to the sensor */
			struct rtio_sqe *read_sqe = rtio_fifo_acquire(ez_io.sq);
			rtio_sqe_prep_read(read_sqe, sensor_dev, RTIO_PRIO_HIGH, buf, ICM20649_RTIO_BUF_SIZE);
	        rtio_fifo_produce(ez_io.sq);
		}
	}

API Reference
*************

RTIO API
========

.. doxygengroup:: rtio_api

RTIO FIFO API
=============

.. doxygengroup:: rtio_fifo
