.. _rtio_interface:

Real-Time IO
############

The real-time I/O or RTIO subsystem exposes an API for efficiently
moving information in a latency and processing friendly way, between
a Zephyr application and external I2C or SPI devices (such as sensors).

Overview
********

RTIO provides an API for allocating and describing continguous blocks of
memory as well as an API for sourcing these blocks from a physical device.

Blocks, defined by :c:type:`struct rtio_block`, provide the unit of data
being transferred. These blocks contain a byte layout identifier provided by
the source of the block, timestamps for when the block
began and finished being created, as well as the length and size in bytes.

Blocks are used to allow for simple DMA transfers as well as controlling the
latency between a source and sink.

Sources can be sensors such has a high rate accelerometer, gyro, or
magnetometer. In these cases the driver will typically provide blocks
encoded in the sensors native format which is often times the most compact
representation.

Intermediary processing steps representing both sinks and sources of data are
also possible. One possible task is taking the raw byte layout coming from a
device such as a 9-DoF intertial motion sensors and converting it to SI units
with common numerical formats for that sensor kind. Several processing steps
might be connected together. Taking the above example of getting a common
9-DoF SI data stream could be used to determine an earth orientation vector
using sensor fusion.

The RTIO subsystem lets you both efficiently move data around and
efficiently process chunks of data at a time.

API Reference
*************

.. doxygengroup:: rtio_interface
   :project: Zephyr
