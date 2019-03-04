#ifndef ZEPHYR_LIS3MDL_H_
#define ZEPHYR_LIS3MDL_H_

int lis3mdl_sleep(struct device *dev);
int lis3mdl_wakeup(struct device *dev);

#endif
