.. _lis3mdl:

LIS3MDL: Magnetometer sensor Monitor
###########################

Overview
********
This sample sets the LIS3MDL magnetometer to its default sampling rate
and enable a trigger on data ready. It displays on the console the
values for magnetometer.

Requirements
************

This sample uses the LIS3MDL sensor controlled using the I2C or SPI interface.
It has been tested on both :ref:`nrf52840_pca10056` board.

References
**********

- LIS3MDL http://www.st.com/en/mems-and-sensors/lis3mdl.html

Building and Running
********************

 This project outputs sensor data to the console. It requires an LIS3MDL
 sensor be connected to a board using pins described in the dts overlay file.

Sample Output
=============

.. code-block:: console

    LIS3MDL sensor samples:

    magn (-0.042000 0.294000 -0.408000) gauss
    - (0) (trig_cnt: 190474)

    <repeats endlessly every 2 seconds>
