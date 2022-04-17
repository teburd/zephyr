/*
 * Copyright (c) 2022 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drivers/sensor_v2.h
 *
 * @brief Public APIs for the sensor driver.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_V2_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_V2_H_

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_
#error "Please include drivers/sensor.h directly"
#endif

/**
 * @brief Sensor Interface V2
 * @defgroup sensor_interface Sensor Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor_types.h>
#include <zephyr/drivers/sensor_utils.h>
#include <zephyr/math/util.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The units in which a sensor represents its range.
 */
enum sensor_range_units {
	/** Gravity units, may be used in SENSOR_TYPE_ACCELEROMETER */
	SENSOR_RANGE_UNITS_ACCEL_G,
	/** \f$m/s^2\f$, may be used in SENSOR_TYPE_ACCELEROMETER */
	SENSOR_RANGE_UNITS_ACCEL_MS2,
	/** Radians, may be used in SENSOR_TYPE_GYROSCOPE */
	SENSOR_RANGE_UNITS_ANGLE_RADIANS,
	/** Rotational degrees, may be used in SENSOR_TYPE_GYROSCOPE */
	SENSOR_RANGE_UNITS_ANGLE_DEGREES,
	/** &micro;Tesla, may be used in SENSOR_TYPE_GEOMAGNETIC_FIELD */
	SENSOR_RANGE_UNITS_GEOMAGNETIC_MICRO_TESLAS,
	/** Pascals, may be used in SENSOR_TYPE_PRESSURE */
	SENSOR_RANGE_UNITS_PRESSURE_PASCALS,
	/** Lux, may be used in SENSOR_TYPE_LIGHT */
	SENSOR_RANGE_UNITS_LIGHT_LUX,
	/**
	 * Celsius, may be used in:
	 * - SENSOR_TYPE_ACCELEROMETER_TEMPERATURE
	 * - SENSOR_TYPE_GYROSCOPE_TEMPERATURE
	 * - SENSOR_TYPE_GEOMAGNETIC_FIELD_TEMPERATURE
	 */
	SENSOR_RANGE_UNITS_TEMPERATURE_C,
	/**
	 * Kelvin, may be used in:
	 * - SENSOR_TYPE_ACCELEROMETER_TEMPERATURE
	 * - SENSOR_TYPE_GYROSCOPE_TEMPERATURE
	 * - SENSOR_TYPE_GEOMAGNETIC_FIELD_TEMPERATURE
	 */
	SENSOR_RANGE_UNITS_TEMPERATURE_K,
};

/**
 * @brief Scale metadata required to convert raw samples to other units.
 *
 * By taking a raw uint32_t sample from a sensor along with this metadata it's possible to get the
 * sample in the same units as the range_units via:
 *
 * @code
 * scale  = range / ((1 << resolution) - 1)
 * sample = raw * scale
 * @endcode
 *
 * The resulting sample is in the same units of the range_units field.
 */
struct sensor_scale_metadata {
	/** The units represented by the range field */
	enum sensor_range_units range_units;

	/**
	 * @brief The range of the measurement
	 *
	 * @see sensor_set_range
	 */
	fp_t range;

	/**
	 * @brief The resolution of the measurement (in bits)
	 *
	 * @see sensor_set_resolution
	 */
	uint8_t resolution;
};

/**
 * @brief An instance of a FIFO iterator
 *
 * The iterator is used to extract data from the raw hardware FIFO buffer using the scale of the
 * sensor at the time of reading.
 */
struct sensor_fifo_iterator {
	/** The raw data read from the hardware FIFO */
	const struct sensor_raw_data *data;
	/** The current iterator offset. Do not modify this value. */
	size_t offset;
};

/**
 * @brief A helper macro for assigning an iterator
 *
 * @param raw_data Pointer to the sensor_raw_data containing the data to iterate over
 */
#define SENSOR_FIFO_ITERATOR_OF(raw_data)                                                          \
	{                                                                                          \
		.data = raw_data, .offset = 0,                                                     \
	}

/**
 * @typedef sensor_fifo_iterator_next_t
 * @brief FIFO iterator API function to move the iterator to the next value
 *
 * @see sensor_fifo_iterator_api.next for argument description
 */
typedef int (*sensor_fifo_iterator_next_t)(struct sensor_fifo_iterator *iterator);

/**
 * @typedef sensor_fifo_iterator_get_sensor_type_t
 * @brief FIFO iterator API function to get the sensor type of the current iterator position
 *
 * @see sensor_fifo_iterator_api.get_sensor_type for argument description
 */
typedef int (*sensor_fifo_iterator_get_sensor_type_t)(const struct sensor_fifo_iterator *iterator,
						      uint32_t *sensor_type);

/**
 * @typedef sensor_fifo_iterator_read_t
 * @brief FIFO iterator API function to read the current iterator position into SI units
 *
 * @see sensor_fifo_iterator_api.read for argument description
 */
typedef int (*sensor_fifo_iterator_read_t)(const struct sensor_fifo_iterator *iterator, void *out);

/**
 * @brief The FIFO iterator API as described by the individual sensor
 */
struct sensor_fifo_iterator_api {
	/**
	 * @brief Move the iterator to the next position if possible
	 *
	 * @param iterator The iterator to move
	 * @return 0 if the iterator was moved
	 * @return < 0 on failure
	 */
	sensor_fifo_iterator_next_t next;

	/**
	 * @brief Get the sensor type of the current iterator position
	 *
	 * @param iterator The iterator to query
	 * @param sensor_type Pointer to the sensor type that will be written on success
	 * @return 0 on success
	 * @return < 0 on failure
	 */
	sensor_fifo_iterator_get_sensor_type_t get_sensor_type;

	/**
	 * @brief Read the current iterator position as SI units
	 *
	 * It is the responsibility of the caller to make sure that the out argument type matches up
	 * with the current iterator's sensor type. This means that if get_sensor_type returns
	 * SENSOR_TYPE_ACCELEROMETER, then out must be of type sensor_three_axis_data.
	 *
	 * @param iterator The iterator to query
	 * @param out The output data structure to write to
	 * @return 0 on success
	 * @return < 0 on failure
	 */
	sensor_fifo_iterator_read_t read;
};

/**
 * @typedef sensor_read_data_t
 * @brief Sensor API function for reading a single data point
 *
 * @see sensor_read_data() for argument description
 */
typedef int (*sensor_read_data_t)(const struct device *sensor, uint32_t sensor_type, void *data);

/**
 * @typedef sensor_get_scale_t
 * @brief Sensor API function for getting the current scale
 *
 * @see sensor_get_scale() for argument description
 */
typedef int (*sensor_get_scale_t)(const struct device *sensor, uint32_t sensor_type,
				  struct sensor_scale_metadata *scale);

/**
 * @typedef sensor_set_range_t
 * @brief Sensor API function for setting the current range
 *
 * @see sensor_set_range() for argument description
 */
typedef int (*sensor_set_range_t)(const struct device *sensor, uint32_t sensor_type, fp_t range,
				  bool round_up);

/**
 * @typedef sensor_get_bias_t
 * @brief Sensor API function for getting the current bias and the temperature at which it was set
 *
 * @see sensor_get_bias() for argument description
 */
typedef int (*sensor_get_bias_t)(const struct device *sensor, uint32_t sensor_type,
				 int16_t *temperature, fp_t *bias_x, fp_t *bias_y, fp_t *bias_z);

/**
 * @typedef sensor_set_bias_t
 * @brief Sensor API function for setting the bias and the temperature at which it was attained
 *
 * @see sensor_set_bias() for argument description
 */
typedef int (*sensor_set_bias_t)(const struct device *sensor, uint32_t sensor_type,
				 int16_t temperature, fp_t bias_x, fp_t bias_y, fp_t bias_z,
				 bool round_up);

/**
 * @typedef sensor_set_resolution_t
 * @brief Sensor API function for setting the current resolution in bits/sample
 *
 * @see sensor_set_resolution() for argument description
 */
typedef int (*sensor_set_resolution_t)(const struct device *sensor, uint32_t sensor_type,
				       uint8_t resolution, bool round_up);

/**
 * @typedef sensor_process_data_callback_t
 * @brief Callback type for handling asynchronous data
 *
 * This callback will be called once per data generated by the sensor in an asynchronous manner.
 * Usually this means that the sensor is using a hardware FIFO and data was being read. There will
 * be one call per data event. The data pointer (set by sensor_set_fifo_data_buffer()) will contain
 * the readings. The number of readings can be obtained via data->reading_count, while the
 * individual readings can be attained by accessing the data->readings[] array. Note that each
 * reading has the sensor type associated with it to allow mixed buffers.
 *
 * @param sensor Pointer to the sensor device
 * @param data The data generated
 * @return 0 on success
 * @return < 0 on failure
 *
 * @see sensor_set_process_data_callback() for use.
 */
typedef int (*sensor_process_data_callback_t)(const struct device *sensor,
					      struct sensor_raw_data *data);

/**
 * @typedef sensor_set_fifo_data_buffer_t
 * @brief Sensor API function for setting the data buffer for FIFO reads
 *
 * @see sensor_set_fifo_data_buffer() for argument description
 */
typedef int (*sensor_set_fifo_data_buffer_t)(const struct device *sensor,
					     struct sensor_raw_data *buffer);

/**
 * @typedef sensor_set_process_data_callback_t
 * @brief Sensor API function for setting the asynchronous callback handler function
 *
 * @see sensor_set_process_data_callback() for argument description
 */
typedef int (*sensor_set_process_data_callback_t)(const struct device *sensor,
						  sensor_process_data_callback_t callback);

/**
 * @typedef sensor_flush_fifo_t
 * @brief Sensor API function for flushing the hardware FIFO
 *
 * @see sensor_flush_fifo() for argument description
 */
typedef int (*sensor_flush_fifo_t)(const struct device *sensor);

/**
 * @typedef sensor_get_fifo_iterator_api_t
 * @brief Sensor API function to get the iterator API for the FIFO data
 *
 * @see sensor_get_fifo_iterator_api() for argument description
 */
typedef int (*sensor_get_fifo_iterator_api_t)(const struct device *sensor,
					      struct sensor_fifo_iterator_api *api);

#define BUILD_SAMPLE_RATE_INFO(type, mhz)                                                          \
	{                                                                                          \
		.sensor_type = (type), .sample_rate_mhz = (mhz),                                   \
	}

/** A single entry of sample rate allowed on the device. */
struct sensor_sample_rate_info {
	uint32_t sensor_type;	 /**< The sensor type that this sample rate applies to */
	uint32_t sample_rate_mhz; /**< The sample rate in mHz */
};

/**
 * @typedef sensor_get_sample_rate_available_t
 * @brief Sensor API function for getting the various available sensor rates
 *
 * @see sensor_get_sample_rate_available() for argument description
 */
typedef int (*sensor_get_sample_rate_available_t)(
	const struct device *sensor, const struct sensor_sample_rate_info **sample_rates,
	uint8_t *count);

/**
 * @typedef sensor_get_sample_rate_t
 * @brief Sensor API function for getting the current sample rate
 *
 * @see sensor_get_sample_rate() for argument description
 */
typedef int (*sensor_get_sample_rate_t)(const struct device *sensor, uint32_t sensor_type,
					uint32_t *sample_rate);

/**
 * @typedef sensor_set_sample_rate_t
 * @brief Sensor API function for setting the current sample rate
 *
 * @see sensor_set_sample_rate() for argument description
 */
typedef int (*sensor_set_sample_rate_t)(const struct device *sensor, uint32_t sensor_type,
					uint32_t sample_rate, bool round_up);

/**
 * @typedef sensor_fifo_get_watermark_t
 * @brief Sensor API function for getting the current FIFO watermark
 *
 * @see sensor_fifo_get_watermark() for argument description
 */
typedef int (*sensor_fifo_get_watermark_t)(const struct device *sensor, uint8_t *watermark_percent);

/**
 * @typedef sensor_fifo_set_watermark_t
 * @brief Sensor API function for setting the current FIFO watermark
 *
 * @see sensor_fifo_set_watermark() for argument description
 */
typedef int (*sensor_fifo_set_watermark_t)(const struct device *sensor, uint8_t watermark_percent,
					   bool round_up);

/**
 * @typedef sensor_fifo_get_streaming_mode_t
 * @brief Sensor API function for checking if streaming mode is enabled in the FIFO
 *
 * @see sensor_fifo_get_streaming_mode() for argument description
 */
typedef int (*sensor_fifo_get_streaming_mode_t)(const struct device *sensor, bool *enabled);

/**
 * @typedef sensor_fifo_set_streaming_mode_t
 * @brief Sensor API function for enabling/disabling the FIFO streaming mode
 *
 * @see sensor_fifo_set_streaming_mode() for argument description
 */
typedef int (*sensor_fifo_set_streaming_mode_t)(const struct device *sensor, bool enabled);

/**
 * @typedef sensor_perform_calibration_t
 * @brief Sensor API function for enabling/disabling live calibration
 *
 * @see sensor_perform_calibration() for argument description
 */
typedef int (*sensor_perform_calibration_t)(const struct device *sensor, bool enable);

__subsystem struct sensor_driver_api_v2 {
	sensor_read_data_t read_data;
	sensor_get_scale_t get_scale;
	sensor_set_range_t set_range;
	sensor_set_resolution_t set_resolution;
	sensor_get_bias_t get_bias;
	sensor_set_bias_t set_bias;
#ifdef CONFIG_SENSOR_STREAMING_MODE
	sensor_set_fifo_data_buffer_t set_fifo_data_buffer;
	sensor_set_process_data_callback_t set_process_data_callback;
	sensor_flush_fifo_t flush_fifo;
	sensor_get_fifo_iterator_api_t get_fifo_iterator_api;
	sensor_get_sample_rate_available_t get_sample_rate_available;
	sensor_get_sample_rate_t get_sample_rate;
	sensor_set_sample_rate_t set_sample_rate;
	sensor_fifo_get_watermark_t get_watermark;
	sensor_fifo_set_watermark_t set_watermark;
	sensor_fifo_get_streaming_mode_t get_streaming_mode;
	sensor_fifo_set_streaming_mode_t set_streaming_mode;
#endif /* CONFIG_SENSOR_STREAMING_MODE */
#ifdef CONFIG_SENSOR_HW_CALIBRATION
	sensor_perform_calibration_t perform_calibration;
#endif /* CONFIG_SENSOR_HW_CALIBRATION */
};

/**
 * @brief A common API allowing the application to control sensor processing
 *
 * @note This is only valid if CONFIG_SENSOR_STREAMING_MODE is enabled
 *
 * @param work The work object to schedule
 * @return 0 on success
 * @return -ENOSYS if streaming mode isn't enabled
 * @return Other < 0 values on failure
 */
int sensor_submit_work(struct k_work *work);

/**
 * @brief Read sensor data.
 *
 * Read a single sample from the sensor. The sample is of a single type and a single sensor data
 * collection entry. If multiple samples are required at the same time, the FIFO API should be used
 * instead.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read (must be one of SENSOR_TYPE_* or a custom vendor
 *        specific value).
 * @param data Pointer to the data that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_read_data(const struct device *sensor, uint32_t sensor_type, void *data);

static inline int z_impl_sensor_read_data(const struct device *sensor, uint32_t sensor_type,
					  void *data)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->read_data == NULL || data == NULL) {
		return -ENOSYS;
	}

	return api->read_data(sensor, sensor_type, data);
}

/**
 * @brief Get the scale of the sensor
 *
 * Get the current operating scale metadata of the sensor.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param scale Pointer to the output scale metadata variable that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_get_scale(const struct device *sensor, uint32_t sensor_type,
			       struct sensor_scale_metadata *scale);

static inline int z_impl_sensor_get_scale(const struct device *sensor, uint32_t sensor_type,
					  struct sensor_scale_metadata *scale)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_scale == NULL) {
		return -ENOSYS;
	}

	return api->get_scale(sensor, sensor_type, scale);
}

/**
 * @brief Set the range of the sensor
 *
 * Update the range of the sensor based on the sensor type's units (SI units).
 *
 * It is important to note that generally, as the range increases, accuracy will decrease.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param range The new range to use
 * @param round_up Whether or not to round up to the nearest available range
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_range(const struct device *sensor, uint32_t sensor_type, fp_t range,
			       bool round_up);

static inline int z_impl_sensor_set_range(const struct device *sensor, uint32_t sensor_type,
					  fp_t range, bool round_up)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_range == NULL) {
		return -ENOSYS;
	}

	return api->set_range(sensor, sensor_type, range, round_up);
}

/**
 * @brief Get the latest bias for a three axis sensor
 *
 * Get the latest bias set for this sensor. It is important to take note of the temperature at which
 * the bias was set, this will help determine how accurate the bias is.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param temperature Pointer to the temperature variable that will be set on success, INT16_MIN if
 *        not supported
 * @param bias_x Pointer to the X component of the bias that will be set on success
 * @param bias_y Pointer to the Y component of the bias that will be set on success
 * @param bias_z Pointer to the Z component of the bias that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_get_bias(const struct device *sensor, uint32_t sensor_type,
			      int16_t *temperature, fp_t *bias_x, fp_t *bias_y, fp_t *bias_z);

static inline int z_impl_sensor_get_bias(const struct device *sensor, uint32_t sensor_type,
					 int16_t *temperature, fp_t *bias_x, fp_t *bias_y,
					 fp_t *bias_z)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_bias == NULL) {
		return -ENOSYS;
	}

	return api->get_bias(sensor, sensor_type, temperature, bias_x, bias_y, bias_z);
}

/**
 * @brief Manually set the bias for a three axis sensor
 *
 * Update the sensor's bias. This bias can be used to account for slight changes in the sensor's
 * accuracy (especially when the sensor's internal temperature varies).
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param temperature The temperature at which the bias was calculated
 * @param bias_x The X component of the bias
 * @param bias_y The Y component of the bias
 * @param bias_z The Z component of the bias
 * @param round_up Whether or not to round up the bias values
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_bias(const struct device *sensor, uint32_t sensor_type,
			      int16_t temperature, fp_t bias_x, fp_t bias_y, fp_t bias_z,
			      bool round_up);

static inline int z_impl_sensor_set_bias(const struct device *sensor, uint32_t sensor_type,
					 int16_t temperature, fp_t bias_x, fp_t bias_y, fp_t bias_z,
					 bool round_up)
{
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_bias == NULL) {
		return -ENOSYS;
	}

	return api->set_bias(sensor, sensor_type, temperature, bias_x, bias_y, bias_z, round_up);
}

/**
 * @brief Update the sensor's resolution
 *
 * Set the sensor's resolution in terms of the number of bits per sample.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param resolution The new resolution measured in number of bits per sample
 * @param round_up Whether or not to round up to the nearest available resolution
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_resolution(const struct device *sensor, uint32_t sensor_type,
				    uint8_t resolution, bool round_up);

static inline int z_impl_sensor_set_resolution(const struct device *sensor, uint32_t sensor_type,
					       uint8_t resolution, bool round_up)
{
#ifdef CONFIG_SENSOR_RESOLUTION
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_resolution == NULL) {
		return -ENOSYS;
	}

	return api->set_resolution(sensor, sensor_type, resolution, round_up);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Set the buffer to use for the next FIFO read
 *
 * @param sensor Pointer to the sensor device
 * @param buffer The buffer to use
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_fifo_data_buffer(const struct device *sensor,
					  struct sensor_raw_data *buffer);

static inline int z_impl_sensor_set_fifo_data_buffer(const struct device *sensor,
						     struct sensor_raw_data *buffer)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_fifo_data_buffer == NULL) {
		return -ENOSYS;
	}

	return api->set_fifo_data_buffer(sensor, buffer);
#else
	return -ENOSYS;
#endif
}
/**
 * @brief Set the processing function for new data.
 *
 * Set the sensor's function for processing data. This callback is important
 * because sensors which make use of a FIFO should not also need to cache the data
 * in memory while waiting. Instead, these sensors will rely on this callback to
 * notify the application of the new data. If the application did not set the
 * callback, the data will be discarded.
 *
 * @param sensor Pointer to the sensor device
 * @param callback A callback function to use when new data is available
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_process_data_callback(const struct device *sensor,
					       sensor_process_data_callback_t callback);

static inline int z_impl_sensor_set_process_data_callback(const struct device *sensor,
							  sensor_process_data_callback_t callback)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_process_data_callback == NULL) {
		return -ENOSYS;
	}

	return api->set_process_data_callback(sensor, callback);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Flush the sensor's hardware FIFO
 *
 * Attempt to flush the FIFO. Calling this function will asynchronously trigger the callback
 * function set in sensor_set_process_data_callback(). It will write data to the buffer set in
 * sensor_set_fifo_data_buffer()
 *
 * @param sensor Pointer to the sensor device
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_flush_fifo(const struct device *sensor);

static inline int z_impl_sensor_flush_fifo(const struct device *sensor)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->flush_fifo == NULL) {
		return -ENOSYS;
	}

	return api->flush_fifo(sensor);
#else
	return -ENOSYS
#endif
}

/**
 * @brief Get the FIFO iterator API for this sensor
 *
 * @param sensor Pointer to the sensor device
 * @param api Pointer to the API struct that will be filled
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_get_fifo_iterator_api(const struct device *sensor,
					   struct sensor_fifo_iterator_api *api);

static inline int z_impl_sensor_get_fifo_iterator_api(const struct device *sensor,
						      struct sensor_fifo_iterator_api *api)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *sensor_api = sensor->api;

	if (api == NULL) {
		return -EINVAL;
	}

	if (sensor_api == NULL || sensor_api->get_fifo_iterator_api == NULL) {
		return -ENOSYS;
	}

	return sensor_api->get_fifo_iterator_api(sensor, api);
#else
	return -ENOSYS;
#endif
}
/**
 * @brief Get the available sample rates for the sensor
 *
 * Get the various available sample rates for the sensor (in mHz).
 *
 * @param sensor Pointer to the sensor device
 * @param sample_rates Pointer to an array that will be mapped to the internal list of sample rates.
 * @param count The number of sample_rates
 * @return 0 on success
 * @return < 0 on error

 */
__syscall int sensor_get_sample_rate_available(const struct device *sensor,
					       const struct sensor_sample_rate_info **sample_rates,
					       uint8_t *count);

static inline int
z_impl_sensor_get_sample_rate_available(const struct device *sensor,
					const struct sensor_sample_rate_info **sample_rates,
					uint8_t *count)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_sample_rate_available == NULL) {
		return -ENOSYS;
	}

	return api->get_sample_rate_available(sensor, sample_rates, count);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Get the sensor's sample rate
 *
 * Get the sample rate of the sensor in milli-Hz.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param sample_rate Pointer to the sample rate variable that will be set on
 *        success
 * @return 0 on success
 * @return
 */
__syscall int sensor_get_sample_rate(const struct device *sensor, uint32_t sensor_type,
				     uint32_t *sample_rate);

static inline int z_impl_sensor_get_sample_rate(const struct device *sensor, uint32_t sensor_type,
						uint32_t *sample_rate)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_sample_rate == NULL) {
		return -ENOSYS;
	}

	return api->get_sample_rate(sensor, sensor_type, sample_rate);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Update the sensor's sample rate
 *
 * Set a new sample rate for the sensor. Sample rates should be in milli-Hz.
 *
 * @param sensor Pointer to the sensor device
 * @param sensor_type The type of sensor to read
 * @param sample_rate The new sample rate in milli-Hz
 * @param round_up Whether or not to round up to the nearest available sample rate
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_set_sample_rate(const struct device *sensor, uint32_t sensor_type,
				     uint32_t sample_rate, bool round_up);

static inline int z_impl_sensor_set_sample_rate(const struct device *sensor, uint32_t sensor_type,
						uint32_t sample_rate, bool round_up)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_sample_rate == NULL) {
		return -ENOSYS;
	}

	return api->set_sample_rate(sensor, sensor_type, sample_rate, round_up);
#else
	return -ENOSYS;
#endif
}
/**
 * @brief Get the sensor FIFO watermark value as a percent
 *
 * Get the watermark value of the sensor's FIFO. Once the FIFO is filled to the watermark percent,
 * the sensor will trigger the GPIO interrupt.
 *
 * @param sensor Pointer to the sensor device
 * @param watermark_percent Pointer to the watermark variable that will be set on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_get_watermark(const struct device *sensor, uint8_t *watermark_percent);

static inline int z_impl_sensor_fifo_get_watermark(const struct device *sensor,
						   uint8_t *watermark_percent)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_watermark == NULL) {
		return -ENOSYS;
	}

	return api->get_watermark(sensor, watermark_percent);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Set the sensor FIFO watermark value as a percent
 *
 * Set the watermark value of the sensor's FIFO. Once the FIFO is filled to the watermark percent,
 * the sensor will trigger the GPIO interrupt.
 *
 * @param sensor Pointer to the sensor device
 * @param watermark_percent The new watermark value to set as a percent.
 * @param round_up True to round up the watermark to the nearest supported percent, false to round
 *        down.
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_set_watermark(const struct device *sensor, uint8_t watermark_percent,
					bool round_up);

static inline int z_impl_sensor_fifo_set_watermark(const struct device *sensor,
						   uint8_t watermark_percent, bool round_up)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_watermark == NULL) {
		return -ENOSYS;
	}

	return api->set_watermark(sensor, watermark_percent, round_up);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Get the streaming/normal mode of the FIFO
 *
 * Get the current FIFO mode of operation. See sensor_fifo_set_streaming_mode() for more details.
 *
 * @param sensor Pointer to the sensor device
 * @param enabled Pointer to the value which will be written on success
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_get_streaming_mode(const struct device *sensor, bool *enabled);

static inline int z_impl_sensor_fifo_get_streaming_mode(const struct device *sensor, bool *enabled)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->get_streaming_mode == NULL) {
		return -ENOSYS;
	}

	return api->get_streaming_mode(sensor, enabled);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Set the streaming/normal mode of the FIFO
 *
 * When streaming mode is enabled, then the oldest entries in the FIFO are removed to make room for
 * new entries. Otherwise, new samples are discarded.
 *
 * @param sensor Pointer to the sensor device
 * @param enabled Set to true to enable streaming mode, false otherwise
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_fifo_set_streaming_mode(const struct device *sensor, bool enabled);

static inline int z_impl_sensor_fifo_set_streaming_mode(const struct device *sensor, bool enabled)
{
#ifdef CONFIG_SENSOR_STREAMING_MODE
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->set_streaming_mode == NULL) {
		return -ENOSYS;
	}

	return api->set_streaming_mode(sensor, enabled);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Enable or disable calibration for the sensor
 *
 * For sensors that support an internal calibration mode, this function will start or stop that
 * mode.
 *
 * @param sensor Pointer to the sensor device
 * @param enable Whether or not to enable the calibration mode
 * @return 0 on success
 * @return < 0 on failure
 */
__syscall int sensor_perform_calibration(const struct device *sensor, bool enable);

static inline int z_impl_sensor_perform_calibration(const struct device *sensor, bool enable)
{
#ifdef CONFIG_SENSOR_HW_CALIBRATION
	const struct sensor_driver_api_v2 *api = sensor->api;

	if (api == NULL || api->perform_calibration == NULL) {
		return -ENOSYS;
	}

	return api->perform_calibration(sensor, enabled);
#else
	return -ENOSYS;
#endif
}

/**
 * @brief Convert a single sample to sensor scaled units.
 *
 * This function performs the operations needed to convert the raw (x, y, z) values to the scaled
 * values described in scale_metadata. This is done in-place so 'data' is expected to be in raw
 * form. Meaning the direct values set by the driver. When complete, the 'values' or 'x', 'y', and
 * 'z' fields will come valid instead of the raw values.
 *
 * @param scale_metadata The scaling metadata needed to do the conversion
 * @param data The raw sensor data (split into x, y, and z)
 * @param readings_offset The offset into the data's readings array
 */
static inline void
sensor_sample_to_three_axis_data(const struct sensor_scale_metadata *scale_metadata,
				 struct sensor_three_axis_data *data, size_t readings_offset)
{
	fp_t resolution = INT_TO_FP((1 << (scale_metadata->resolution - 1)) - 1);
	fp_t out[3];
#ifdef CONFIG_FPU
	fp_t scale = fp_div(resolution, scale_metadata->range);
#endif

	for (int i = 0; i < 3; ++i) {
		fp_t value = INT_TO_FP(data->readings[readings_offset].raw_values[i]);

#ifdef CONFIG_FPU
		out[i] = fp_mul(value, scale);
#else
		/*
		 * The order here is important, the value must be divided by the resolution first
		 * otherwise the fp_t may overflow when fixed point is used. This means that we lose
		 * the optimization of calculating the scale only once.
		 */
		out[i] = fp_mul(fp_div(value, resolution), scale_metadata->range);
#endif
	}

	for (int i = 0; i < 3; ++i) {
		data->readings[readings_offset].values[i] = out[i];
	}
}

/**
 * @}
 */

#include <syscalls/sensor_v2.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_V2_H_ */
