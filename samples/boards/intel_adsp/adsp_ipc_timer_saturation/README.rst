.. _adsp_ipc_timer_saturation:

ADSP IPC and Timer Saturation
#############################

Overview
********

A simple sample overloads the processing abilities of the DSP with IPC and Timer interrupts
that need to be executed in a timely manner running showing the behavior.

Building and Running
********************

This application can be built and executed on QEMU as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/intel_adsp/adsp_ipc_timer_saturation
   :host-os: unix
   :board: intel_adsp_ace15_mtpm
   :goals: run
   :compact:

To build for another board, change "intel_adsp_ace15_mtpm" above to that adsp board's name.

The image must then be signed

e.g.
.. code-block:: console
	west sign --tool-data=$HOME/csof/sof/rimage/config --tool-path=$HOME/csof/build-rimage/rimage -t rimage -- -k ~/keys/signing_key.pem

For ACE the image must be resized

.. code-block:: console
	truncate -s 512k build/zephyr/zephyr.ri

Then the firmware can be loaded with cavstool.py on cavs or acetool.py on ace.

Sample Output
=============

Produces a fixed bucket histogram describing percentile [99.9, 99, 95, 90, 75, 50] percentile
values all in microseconds for various durations and latencies.

.. code-block:: console

	ISR Duration: [.....]
    Timer Latency: [...]
	Timer ISR Duration: [...]
	IPC ISR Duration: [...]

Exit QEMU by pressing :kbd:`CTRL+A` :kbd:`x`.
