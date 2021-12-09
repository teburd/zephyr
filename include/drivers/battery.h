/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for the Battery (Fuel Gauge, Charger, and Managment) drivers.
 *
 * Battery management may involve one or more devices along with a physical battery to
 * monitor the state of the battery, manage its state of charge, and estimate time to charge
 * or lose power based on its current draw rate.
 *
 * Commonly used information is available using simple functions that wrap a few of the battery
 * driver APIs.
 *
 * Extended information or controls may be available with a particular device implementation
 * header such as #include <drivers/battery/sbs.h> for the Smart Battery API.
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_
#define ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_

/**
 * @brief Battery Interface
 * @defgroup battery_interface Battery Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <device.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A min/max acceptable range used for describing the battery policy
 */
struct battery_range {
	uint32_t min;
	uint32_t max;
};

/**
 * @brief Battery measurements
 */
enum battery_measurement {
	/* Voltage in uV */
	BATTERY_VOLTAGE,
	/* Current in uA */
	BATTERY_CURRENT,
	/* Power in uWh */
	BATTERY_POWER,
	/* Charge in uAh */
	BATTERY_CHARGE,
	/* Temperature in centikelvin */
	BATTERY_TEMP,
	/* Cycle count */
	BATTERY_CYCLES,
	/* Capacity in uAh */
	BATTERY_CAPACITY,
	/* State of Charge in tenths of a percent, (CHARGE*1000)/CAPACITY */
	BATTERY_SOC,
	/* Time to fully discharge in minutes */
	BATTERY_DISCHARGE_TIME,
	/* Time to fully charge in minutes */
	BATTERY_CHARGE_TIME,
	/* Design Capacity in uAh */
	BATTERY_DESIGN_CAPACITY,
	/* Design Voltage in uV */
	BATTERY_DESIGN_VOLTAGE
	/* Absolute SoC in tenths of a percent, (CHARGE*1000)/DESIGN_CAPACITY  */
	BATTERY_ABSOLUTE_SOC
};

/**
 * @brief Battery Policy Settings
 *
 * Describes the ranges or min/max values for status/alarm notification by the battery
 */
struct battery_policy {
	struct battery_range voltage;
	struct battery_range current;
	struct battery_range charge;
	struct battery_range temp;
	uint32_t cycles;
	struct battery_range soc;
	uint32_t discharge_time;
	uint32_t charge_time;
};

/**
 *@brief Battery event bits
 *
 * A set of bit mask definitions for alarms and events that may be set by a battery
 * @{
 **/

/** The battery voltage is above or below the specified range */
#define BATTERY_VOLTAGE_ALARM BIT(0)
/** The battery current is above or below the specified range */
#define BATTERY_CURRENT_ALARM BIT(1)
/** The battery charge is above or below the specified range */
#define BATTERY_CHARGE_ALARM BIT(2)
/** The battery temperature is above or below the specified range */
#define BATTERY_TEMP_ALARM BIT(3)
/** The cycle count has exceeded the specified value */
#define BATTERY_CYCLE_ALARM BIT(4)
/** The time to discharge is less than the set minimum time remaining */
#define BATTERY_DISCHARGE_TIME_ALARM BIT(5)

/** State set when the battery is present */
#define BATTERY_PRESENT_STATE BIT(8)
/** State set when the battery is detected to be charging, or cleared when discharging */
#define BATTERY_CHARGING_STATE BIT(9)
/** State set when the battery is fully charged */
#define BATTERY_CHARGED_STATE BIT(10)
/** State set when the battery is fully discharged */
#define BATTERY_DISCHARGED_STATE BIT(11)

/** @} */

/**
 * @typedef battery_measurement_t
 * @brief Callback API upon getting a measurement from the battery's
 *
 * See battery_measurement() for usage
 */
typedef int32_t (*battery_measurement_t)(const struct device *dev,
					 const enum battery_measurement measurement,
					 int32_t *val);

/**
 * @typedef battery_set_policy_t
 * @brief Callback API upon setting a battery's policy for alarms and status
 *
 * See battery_set_policy() for usage
 */
typedef int32_t (*battery_set_policy_t)(const struct device *dev,
					const struct battery_policy *policy);

/**
 * @typedef battery_status_t
 * @brief Callback API upon getting a battery's status
 *
 * See battery_status() for usage
 */
typedef struct k_event * (*battery_status_t)(const struct device *dev);

/**
 * @brief Battery driver API
 */
__subsystem struct battery_driver_api {
	battery_measurement_t measurement;
	battery_set_policy_t set_policy;
	battery_status_t status;
};

/**
 * @brief Get a measurement from the battery
 *
 * Reads an integer measurement from the battery.
 */
__syscall int32_t battery_measurement(const struct device *dev,
				      const enum battery_measurement measurement);

static inline int32_t z_impl_battery_measurement(const struct device *dev)
{
	const struct battery_driver_api *api =
		(const struct battery_driver_api *)dev->api;

	return api->measurement(dev, measurement);
}

/**
 * @brief Set battery policy
 *
 * A battery may intelligently notify when its state changes or when attributes go beyond
 * specified ranges. The ranges and limits to notify upon are set with a policy.
 *
 * For example if a minimum and maximum voltage range are requested and the voltage
 * of the battery goes below or above the allowed range, an alarm bit is set
 * and any listeners to the k_event object that are setup to watch that bit will be woken.
 *
 * @param dev Battery device to set the event policy for.
 * @param policy Battery event policy
 *
 * @return 0 on Success
 *         -ENOSUP if unsupported
 *         -EINVAL if a parameter is invalid with a logged message describing what.
 */
__syscall int battery_set_policy(const struct device *dev, struct battery_policy *pol);

static inline int z_impl_battery_set_policy(const struct device *dev, struct battery_policy *pol)
{
	const struct battery_driver_api *api =
		(const struct battery_driver_api *)dev->api;

	return api->set_policy(dev, pol);
}

/**
 * @brief Get the status of the battery
 *
 * @param dev Battery device
 *
 * @return k_event object that may be used to inspect and listen for battery alarms
 *         and state changes
 */
__syscall struct k_event *battery_status(const struct device *dev);

static inline struct k_event *z_impl_battery_status(const struct device *dev)
{
	const struct battery_driver_api *api =
		(const struct battery_driver_api *)dev->api;

	return api->status(dev);
}

/**
 * @brief Get the battery voltage in uV
 *
 * The voltage may be be the discharge or charge voltage,
 * depending on the state of the battery. It is always
 * positive.
 *
 * @return voltage on success, -ENOSUP if unsupported
 */
static inline int32_t battery_voltage(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_VOLTAGE);
}

/**
 * @brief Get the battery current in uA
 *
 * The current may be the discharge or charge current,
 * depending on the state of the battery. It is always
 * positive.
 *
 * @return current on success, -ENOSUP if unsupported
 */
static inline int32_t battery_current(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_CURRENT);
}

/**
 * @brief Get the battery charge in uAh
 *
 * @return charge on success, -ENOSUP if unsupported
 */
static inline int32_t battery_charge(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_CHARGE);
}

/**
 * @brief Get the battery power draw in uW
 *
 * @return power on success, -ENOSUP if unsupported
 */
static inline battery_power(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_POWER);
}

/**
 * @brief Get the battery capacity in uAh
 *
 * This is meant to represent the current capacity of the battery
 * if known, or the design capacity if not.
 *
 * @return capacity on success, -ENOSUP if unsupported
 */
static inline int32_t battery_capacity(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_CAPACITY);
}

/**
 * @brief Get the battery temperature in centikelvin (0.01*K)
 *
 * @return temperature on success, -ENOSUP if unsupported
 */
static inline int32_t battery_temp(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_TEMP);
}

/**
 * @brief Get the battery cycle count if available
 *
 * @return The number of charge/discharge cycles the battery
 * has gone through. -ENOSUP if unsupported.
 */
static inline int32_t battery_cycles(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_CYCLES);
}

/**
 * @brief Get the battery state of charge as a percentage
 *
 * Returned value is tenths of a percent in integer form.
 *
 * Effectively (battery_charge()*1000)/battery_capacity()
 *
 * To get SoC as a percentage as a float: float fsoc = (float)soc/10.0f;
 *
 * @return Percentage state of charge or -ENOSUP if unsupported
 */
static inline int32_t battery_soc(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_SOC);
}

/**
 * @brief Get the battery estimated time remaining
 *
 * The estimated number of minutes remaining to discharge
 * the battery.
 *
 * @return Estimated time or -ENOSUP if unsupported.
 */
static inline int32_t battery_discharge_time(const struct device *dev)
{
	return battery_measurement(dev, BATTERY_DISCHARGE_TIME);
}

#ifdef __cplusplus
}
#endif

#include <syscalls/battery.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_BATTERY_H_ */
