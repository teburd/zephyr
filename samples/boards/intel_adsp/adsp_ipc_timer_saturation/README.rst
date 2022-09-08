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
   :board: intel_adsp_cavs25
   :goals: run
   :compact:

To build for another board, change "intel_adsp_cavs25" above to that adsp board's name.

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
