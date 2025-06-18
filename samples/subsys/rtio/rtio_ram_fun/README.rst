RTIO RAM Fun
############

A simple application that can be built in a few ways to let people compare and contrast RTIO vs the traditional
IRQ offloading to a thread in a few scenarios.

Button triggering an I2C transfer a Thread waits on.

Hardware Setup
==============
- Target supporting I2C natively with RTIO, e.g: mimxrt1010_evk, nrf52840dk, stm32f446re.
- Target has a button to trigger on
- External I2C RAM chipset connected to the I2C bus, e.g: Fujitsu's MB85 FeRAM.
- Scope or logic analyzer connect to button line, and i2c clock line
