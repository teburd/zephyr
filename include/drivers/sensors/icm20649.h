#ifndef ZEPHYR_ICM20649_H_
#define ZEPHYR_ICM20649_H_

int icm20649_sleep(struct device *dev);
int icm20649_wakeup(struct device *dev);

#endif
