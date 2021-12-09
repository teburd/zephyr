/*
 * Copyright 2020 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(max17055, CONFIG_SENSOR_LOG_LEVEL);

#include "max17055.h"

#define DT_DRV_COMPAT maxim_max17055

/**
 * @brief Read a register value
 *
 * Registers have an address and a 16-bit value
 *
 * @param priv Private data for the driver
 * @param reg_addr Register address to read
 * @param val Place to put the value on success
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17055_reg_read(struct max17055_data *priv, int reg_addr,
			     int16_t *valp)
{
	uint8_t i2c_data[2];
	int rc;

	rc = i2c_burst_read(priv->i2c, DT_INST_REG_ADDR(0), reg_addr, i2c_data, 2);
	if (rc < 0) {
		LOG_ERR("Unable to read register");
		return rc;
	}
	*valp = (i2c_data[1] << 8) | i2c_data[0];

	return 0;
}

static int max17055_reg_write(struct max17055_data *priv, int reg_addr,
			      uint16_t val)
{
	uint8_t i2c_data[2];

	sys_put_le16(val, i2c_data);

	return i2c_burst_write(priv->i2c, DT_INST_REG_ADDR(0), reg_addr, i2c_data, 2);
}

/**
 * @brief Convert current in MAX17055 units to microamps
 *
 * @param rsense_mohms Value of Rsense in milliohms
 * @param val Value to convert (taken from a MAX17055 register)
 * @return corresponding value in microamps
 */
static int32_t current_to_ua(unsigned int rsense_mohms, int16_t val)
{
	return (val * 15625) / (rsense_mohms * 10);
}

/**
 * @brief Convert current in milliamps to MAX17055 units
 *
 * @param rsense_mohms Value of Rsense in milliohms
 * @param val Value in uA to convert
 * @return corresponding value in MAX17055 units, ready to write to a register
 */
static int32_t current_ua_to_max17055(unsigned int rsense_mohms, uint16_t val)
{
	return (val * rsense_mohms * 10) / 15625;
}

/**
 * @brief Convert capacity in MAX17055 units to microamps
 *
 * @param rsense_mohms Value of Rsense in milliohms
 * @param val Value to convert (taken from a MAX17055 register)
 * @return corresponding value in milliamps
 */
static int capacity_to_ua(unsigned int rsense_mohms, int16_t val)
{
	int lsb_units, rem;

	/* Get units for the LSB in uA */
	lsb_units = 5 * 1000000 / rsense_mohms;

	/* Get remaining capacity in uA */
	rem = val * lsb_units;

	return rem;
}

/**
 * @brief Convert capacity in microamphours to MAX17055 units
 *
 * @param rsense_mohms Value of Rsense in milliohms
 * @param val_uha Value in microamphours to convert
 * @return corresponding value in MAX17055 units, ready to write to a register
 */
static int capacity_to_max17055(unsigned int rsense_mohms, uint16_t val_uha)
{
	return val_uha * rsense_mohms / 5;
}

/**
 * @brief Convert voltage in microvolts to MAX17055 units
 *
 * @param val Value in microvolts to convert
 * @return corresponding value in MAX17055 units, ready to write to a register
 */
static int voltage_uv_to_max17055(uint16_t val)
{
	return val * 16 / 1250;
}

/**
 * @brief Battery measurement
 *
 * @param dev MAX17055 device to access
 * @param msmt Measurement number to read
 * @param valp Returns the sensor value read on success
 * @return 0 if successful
 * @return -ENOTSUP for unsupported measurement
 */
static int max17055_measurement(const struct device *dev,
				const enum battery_measurement msmt,
				int32_t *val)
{
	int rc = 0;
	const struct max17055_config *const config = dev->config;
	struct max17055_data *const priv = dev->data;
	int16_t reg_val = 0;

	switch (chan) {
	case BATTERY_VOLTAGE:
		/* Get voltage in uV */
		rc = max17055_reg_read(priv, VCELL, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = reg_val * 1250 / 16;
		break;
	case BATTERY_CURRENT:
		/* Get current in uA */
		rc = max17055_reg_read(priv, AVG_CURRENT,&reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = current_to_ua(config->rsense_mohms, reg_val);
		break;
	case BATTERY_SOC:
		rc = max17055_reg_read(priv, REP_SOC, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = reg_val / 25600;
		break;
	case BATTERY_TEMP:
		rc = max17055_reg_read(priv, INT_TEMP, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = reg_val / 256000;
		break;
	case BATTERY_CAPACITY:
		rc = max17055_reg_read(priv, FULL_CAP_REP, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val= capacity_to_ua(config->rsense_mohms, reg_val);
		break;
	case BATTERY_CHARGE:
		rc = max17055_reg_read(priv, REP_CAP, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = capacity_to_ma(config->rsense_mohms, reg_val);
		break;
	case BATTERY_DISCHARGE_TIME:
		/* Get time in ms */
		rc = max17055_reg_read(priv, TTE, &reg_val);
		if (rc != 0) {
			return rc;
		}
		if (rev_val == 0xffff) {
			*val = 0;
		} else {
			*val= reg_val * 5625;
		}
		break;
	case BATTERY_CHARGE_TIME:
		rc = max17055_reg_read(priv, TTF, &reg_val);
		if (rc != 0) {
			return rc;
		}
		if (reg_val == 0xffff) {
			*val = 0;
		} else {
			/* Get time in ms */
			*val = reg_val * 5625;
		}
		break;
	case BATTERY_CYCLE_COUNT:
		rc = max17055_reg_read(priv, CYCLES, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = reg_val / 10000;
		break;
	case BATTERY_DESIGN_CAPACITY:
		rc = max17055_reg_read(priv, DESIGN_CAP, &reg_val);
		if (rc != 0) {
			return rc;
		}
		*val = capacity_to_ua(config->rsense_mohms, reg_val);
		break;
	case BATTERY_DESIGN_VOLTAGE:
		*val = config->design_voltage;
		break;
	case BATTERY_DESIRED_VOLTAGE:
		*val = config->desired_voltage;
		break;
	case BATTERY_DESIRED_CHARGING_CURRENT:
		*val = config->desired_charging_current;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int max17055_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct max17055_data *priv = dev->data;

	struct {
		int reg_addr;
		int16_t *dest;
	} regs[] = {
		{ VCELL, &priv->voltage },
		{ AVG_CURRENT, &priv->avg_current },
		{ REP_SOC, &priv->state_of_charge },
		{ INT_TEMP, &priv->internal_temp },
		{ REP_CAP, &priv->remaining_cap },
		{ FULL_CAP_REP, &priv->full_cap },
		{ TTE, &priv->time_to_empty },
		{ TTF, &priv->time_to_full },
		{ CYCLES, &priv->cycle_count },
		{ DESIGN_CAP, &priv->design_cap },
	};

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		int rc;

		rc = max17055_reg_read(priv, regs[i].reg_addr, regs[i].dest);
		if (rc != 0) {
			LOG_ERR("Failed to read channel %d", chan);
			return rc;
		}
	}

	return 0;
}

static int max17055_exit_hibernate(struct max17055_data *priv)
{
	LOG_DBG("Exit hibernate");

	if (max17055_reg_write(priv, SOFT_WAKEUP, SOFT_WAKEUP_WAKEUP)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, HIB_CFG, HIB_CFG_CLEAR)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, SOFT_WAKEUP, SOFT_WAKEUP_CLEAR)) {
		return -EIO;
	}

	return 0;
}

static int max17055_write_config(struct max17055_data *priv,
				 const struct max17055_config *const config)
{
	uint16_t design_capacity = capacity_to_max17055(config->rsense_mohms,
							config->design_capacity);
	uint16_t d_qacc = design_capacity / 32;
	uint16_t d_pacc = d_qacc * 44138 / design_capacity;
	uint16_t i_chg_term = current_ua_to_max17055(config->rsense_mohms, config->i_chg_term);
	uint16_t v_empty = voltage_uv_to_max17055(config->v_empty);

	LOG_DBG("Writing configuration parameters");
	LOG_DBG("DesignCap: %u, dQAcc: %u, IChgTerm: %u, VEmpty: %u, dPAcc: %u",
		design_capacity, d_qacc, i_chg_term, v_empty, d_pacc);

	if (max17055_reg_write(priv, DESIGN_CAP, design_capacity)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, D_QACC, d_qacc)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, ICHG_TERM, i_chg_term)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, V_EMPTY, v_empty)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, D_PACC, d_pacc)) {
		return -EIO;
	}
	if (max17055_reg_write(priv, MODEL_CFG, MODELCFG_REFRESH)) {
		return -EIO;
	}

	uint16_t model_cfg = MODELCFG_REFRESH;

	while (model_cfg & MODELCFG_REFRESH) {
		max17055_reg_read(priv, MODEL_CFG, &model_cfg);
		k_sleep(K_MSEC(10));
	}

	return 0;
}

static int max17055_init_config(struct max17055_data *priv,
				const struct max17055_config *const config)
{
	int16_t hib_cfg;

	if (max17055_reg_read(priv, HIB_CFG, &hib_cfg)) {
		return -EIO;
	}

	if (max17055_exit_hibernate(priv)) {
		return -EIO;
	}

	if (max17055_write_config(priv, config)) {
		return -EIO;
	}

	if (max17055_reg_write(priv, HIB_CFG, hib_cfg)) {
		return -EIO;
	}

	return 0;
}

/**
 * @brief initialise the fuel gauge
 *
 * @return 0 for success
 * @return -EIO on I2C communication error
 * @return -EINVAL if the I2C controller could not be found
 */
static int max17055_gauge_init(const struct device *dev)
{
	int16_t tmp;
	struct max17055_data *priv = dev->data;
	const struct max17055_config *const config = dev->config;

	priv->i2c = device_get_binding(config->bus_name);
	if (!priv->i2c) {
		LOG_ERR("Could not get pointer to %s device", config->bus_name);
		return -EINVAL;
	}

	if (max17055_reg_read(priv, STATUS, &tmp)) {
		return -EIO;
	}

	if (!(tmp & STATUS_POR)) {
		LOG_DBG("No POR event detected - skip device configuration");
		return 0;
	}

	/* Wait for FSTAT_DNR to be cleared */
	tmp = FSTAT_DNR;
	while (tmp & FSTAT_DNR) {
		max17055_reg_read(priv, FSTAT, &tmp);
	}

	if (max17055_init_config(priv, config)) {
		return -EIO;
	}

	/* Clear PowerOnReset bit */
	if (max17055_reg_read(priv, STATUS, &tmp)) {
		return -EIO;
	}

	tmp &= ~STATUS_POR;
	return max17055_reg_write(priv, STATUS, tmp);
}

static const struct battery_driver_api max17055_battery_driver_api = {
	.measurement = max17055_measurement,
	.status = max17055_status,
	.set_policy = max17055_set_policy,
};

#define MAX17055_INIT(index)								   \
	static struct max17055_data max17055_driver_##index;				   \
											   \
	static const struct max17055_config max17055_config_##index = {			   \
		.bus_name = DT_INST_BUS_LABEL(index),					   \
		.design_capacity = DT_INST_PROP(index, design_capacity),		   \
		.design_voltage = DT_INST_PROP(index, design_voltage),			   \
		.desired_charging_current = DT_INST_PROP(index, desired_charging_current), \
		.desired_voltage = DT_INST_PROP(index, desired_voltage),		   \
		.i_chg_term = DT_INST_PROP(index, i_chg_term),				   \
		.rsense_mohms = DT_INST_PROP(index, rsense_mohms),			   \
		.v_empty = DT_INST_PROP(index, v_empty),				   \
	};										   \
											   \
	DEVICE_DT_INST_DEFINE(index, &max17055_gauge_init,				   \
			      NULL,							   \
			      &max17055_driver_##index,					   \
			      &max17055_config_##index, POST_KERNEL,			   \
			      CONFIG_BATTERY_INIT_PRIORITY,				   \
			      &max17055_battery_driver_api)

DT_INST_FOREACH_STATUS_OKAY(MAX17055_INIT);
