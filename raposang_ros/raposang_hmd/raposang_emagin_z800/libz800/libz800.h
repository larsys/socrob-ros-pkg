/**
 * @mainpage libz800
 *
 * libz800 is a libusb-based user-space driver for the eMagin Z800 
 * 3D Visor.
 *
 * For information about the API see the following pages:
 *
 * - @ref device_connection "Device connection"
 * - @ref eeprom_access "EEPROM access"
 * - @ref tracker_communication "Tracker communication"
 * - @ref display_communication "Display communication"
 *
 * @page device_connection Device connection
 *
 * All access to the Z800 device is mangaged through the opaque
 * structure z800_device which contains all connection-related 
 * state. A pointer to such structure can be obtained by
 * calling z800_connect(). 
 * 
 * Example:
 * @code

	z800_device *dev = z800_connect(2);
	if(dev == NULL)
	{
		fprintf(stderr, "z800_connect() failed");
	}
	else
	{
		fprintf(stderr, "z800_connect() succeeded");
		z800_disconnect(dev);
	}
@endcode
 *
 * After connection is established the debug level can be 
 * changed using z800_set_debug_level().
 *
 * @page eeprom_access EEPROM access
 *
 * The function z800_get_firmware_version() can be used
 * to check the firmware version stored in EEPROM - further
 * access to the stored data is provided using the following
 * functions:
 * 
 * - z800_get_properties()
 * - z800_get_property_meta_info()
 * - z800_read_property()
 * - z800_write_property()
 *
 * @page tracker_communication Tracker communication
 *
 * When a device pointer is obtained the tracker can be polled
 * for new data with z800_poll_tracker(). On success, this function
 * updates the internal state with the new accelerometer, magnetometer 
 * and gyro measurements.
 *
 * After z800_poll_tracker() has been called the following functions
 * can be called for debugging purposes:
 * - z800_get_raw_tracker_data()
 * - z800_get_gyro_speed()
 * - z800_get_stabilization()
 *
 * Note however for normal operation these functions need <b>not</b> be 
 * called. 
 *
 * A kalman-filtered orientation is accessable as a quaternion (XYZW)
 * using z800_get_orientation().
 * 
 * Example:
 * 
 * @code

	if(z800_poll_tracker(dev) == Z800_FALSE)
	{
		fprintf(stderr, "z800_poll_tracker() failed");
	}
	else
	{
		double q[4];
		if(z800_get_orientation(_dev, &q) == Z800_FALSE)
		{
			fprintf(stderr, "z800_get_orientation() failed");
		}
		else
		{
			printf("Orientation: %+2.f %+2.f %+2.f %+2.f\n", q[0], q[1], q[2], q[3]);			
		}
	}
@endcode
 *
 * @section calibration_data Calibration data
 * 
 * libz800 currently has certain calibration data for the various sensors hard-coded
 * for my specific device. It looks like these values are okay for other devices too but
 * in case they need to be adjusted the following functions are available:
 *
 * - z800_get_calibration_data()
 * - z800_set_calibration_data()
 *
 * @page display_communication Display communication
 *
 * Access to the display is provided using the following functions:
 * 
 * - z800_wake()
 * - z800_set_stereo_mode()
 * - z800_flip_stereo()
 * - z800_step_brightness()
 *
 */

#ifndef LIBZ800_H
#define LIBZ800_H

/**
 * @internal
 *
 * @{
 */

#ifndef Z800_EXPORT
#  ifdef WIN32
#    ifdef INSIDE_Z800
#      define Z800_EXPORT _declspec(dllexport)
#    else
#      define Z800_EXPORT _declspec(dllimport)
#    endif /* Z800_EXPORT */
#  else
#    define Z800_EXPORT
#  endif /* WIN32 */
#endif

/**
 * @}
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Opaque structure representing an active connection to an attached Z800 device, storing
 * all data related to communication and filtering.
 */
typedef struct z800_device z800_device;

typedef enum
{
	Z800_FALSE,
	Z800_TRUE
} z800_bool;

/**
 * Connects to an attached Z800 device.
 *
 * @returns a handle representing the connection on success.
 * @returns NULL on failure.
 */
Z800_EXPORT z800_device *z800_connect(
	int debug_level
);

/**
 * Disconnects from a Z800 device.
 *
 * @param dev Device to disconnect from.
 *
 * @note After calling this function the passed handle is no
 *       longer valid and must not be passed to any other
 *       function.
 */
Z800_EXPORT void z800_disconnect(
	z800_device *dev
);

/**
 * Sets the debug level of a device.
 *
 * @param dev Device to set the debug level off.
 * @param debug_level The debug level to set.
 */
Z800_EXPORT void z800_set_debug_level(
	z800_device *dev, 
	int debug_level
);

/**
 * Retrieves the firmware version of a device.
 *
 * @param dev Device to obtain the firmware version data from.
 *
 * @returns (major << 8) | minor on success.
 * @returns -1 on failure.
 */
Z800_EXPORT int z800_get_firmware_version(
	z800_device *dev
);

/**
 * Awakes a device from standby or resets the standby timer.
 *
 * @param dev Device to wake.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_wake(
	z800_device *dev
);

/**
 * Enables / disables the stereo mode of a device.
 *
 * @param dev Device to enable / disable the stereo mode for.
 * @param enabled Whether to enable or disable the stereo mode.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_set_stereo_mode(
	z800_device *dev, 
	z800_bool enabled
);

/**
 * Flips the left and right eye of a device in stereo mode.
 *
 * @param dev Device to flip the left and right eye for.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_flip_stereo(
	z800_device *dev
);

/**
 * Steps up the display brightness of a device.
 *
 * @param dev Device to step up the brightness for.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_step_brightness(
	z800_device *dev
);

/**
 * Polls the tracker data from a device.
 *
 * @param dev Device to poll the tracker data from.
 *
 * @note This function must be called before z800_get_raw_tracker_data(),
 *       z800_get_gyro_speed(), z800_get_stabilization() and/or z800_get_orientation().
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_poll_tracker(
	z800_device *dev
);

/**
 * Obtains the raw tracker data from the attached device.
 *
 * @param dev Device to obtain the raw tracker data from.
 * @param gyr Pointer to an array of 3 floats to retrieve the normalized gyro speeds (XYZ).
 * @param acc Pointer to an array of 3 floats to retrieve the normalized acceleration vector (XYZ).
 * @param mag Pointer to an array of 3 floats to retrieve the normalized magnetic vector (XYZ).
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 *
 * @note All values are retrieved in undefined units between the range [-1..1]
 */
Z800_EXPORT z800_bool z800_get_raw_tracker_data(
	z800_device *dev, 
	float (*gyr)[3],
	float (*acc)[3], 
	float (*mag)[3]
);

/**
 * Obtains partially processed tracker data from the attached device with the 
 * accelerometer and magnetometer values filtered and calibrated.
 *
 * @param dev Device to obtain the tracker data from.
 * @param gyr Pointer to an array of 3 floats to retrieve the normalized gyro speeds (XYZ).
 * @param acc Pointer to an array of 3 floats to retrieve the normalized, filtered and calibrated acceleration vector (XYZ).
 * @param mag Pointer to an array of 3 floats to retrieve the normalized, filtered and calibrated  magnetic vector (XYZ).
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 *
 * @note All values are retrieved in undefined units between the range [-1..1]
 */
Z800_EXPORT z800_bool z800_get_calibrated_tracker_data(
	z800_device *dev, 
	float (*gyr)[3],
	float (*acc)[3],
	float (*mag)[3]
);

/**
 * @brief Tracker calibration data.
 */
struct z800_calibration_data
{
	float acc_offset[3];
	float acc_gain[3];
	float mag_offset[3];
	float mag_gain[3];
	float gyr_offset[3];
	float gyr_gain[3];
};

/**
 * Obtains the current calibration data from a device.
 *
 * @param dev Device to obtain the calibration data from.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_get_calibration_data(
	z800_device *dev,
	struct z800_calibration_data *data
);

/**
 * Sets the calibration data for a device.
 *
 * @param dev Device to set the calibration data for.
 * @param data Pointer to the new calibration data.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_set_calibration_data(
	z800_device *dev,
	const struct z800_calibration_data *data
);

/**
 * Obtains the (calibrated) gyro speeds in radians per second.
 *
 * @param dev Device to obtain the gyro speeds from.
 * @param gyr Pointer to an array of 3 floats to retrieve the normalized gyro speeds (XYZ).
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 *
 * @note Concerning the calibration performed both offset and gain are calculated into
 *	     the speeds retrieved by this function. Cutoff however is not.
 */
Z800_EXPORT z800_bool z800_get_gyro_speed(
	z800_device *dev, 
	float (*gyr)[3]
);

/**
 * Obtains the measured (combined) magnetometer and accelerometer orientation 
 * used to stabilize the calculated calibration.
 *
 * @param dev Device to obtain the stabilization orientation from.
 * @param q Pointer to an array of 4 floats to retrieve the stabilization 
 *          orientation (XYZW).
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_get_stabilization(
	z800_device *dev, 
	double (*q)[4]
);

/**
 * Obtains the calculcated orientation after calibration and filtering as a quaternion.
 *
 * @param dev Device to obtain the orientation from.
 * @param q Pointer to an array of 4 floats to retrieve the orientation (XYZW).
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_get_orientation(
	z800_device *dev, 
	double (*q)[4]
);

/**
 * Obtains the list of EEPROM properties stored in a device.
 *
 * @param dev Device to obtain the list of properites from.
 *
 * @returns Pointer to a NULL terminated array of strings on success.
 * @returns NULL on failure.
 */
Z800_EXPORT const char * const *z800_get_properties(
	z800_device *dev
);

/**
 * @brief Various meta-information for a Z800 property stored in EEPROM on 
 *        the device.
 */
struct z800_property
{
	/**
	 * Whether or not this property exists separately for the left and
	 * right display.
	 */
	int stereo;

	/**
	 * Minimum value.
	 */
	int min_value;

	/**
	 * Maximum value.
	 */
	int max_value;

	/**
	 * Factory default value.
	 */
	int default_value;
};

/**
 * Obtains (constant) meta-data about a EEPROM property.
 *
 * @param dev Device to obtain the meta-data from.
 * @param name Then name of the property.
 * @param prop Pointer to a z800_property stucture to receive the meta-data.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_get_property_meta_info(
	z800_device *dev, 
	const char *name, 
	struct z800_property *prop
);

/**
 * Reads a EEPROM propery's value from the device.
 *  
 * @param dev Device to read the data from.
 * @param name Then name of the property.
 * @param value Pointer to an integer to store the result.
 * @param right_display Whether to access the right or left's display property.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 */
Z800_EXPORT z800_bool z800_read_property(
	z800_device *dev, 
	const char *name, 
	int *value, 
	z800_bool right_display
);

/**
 * Writes a EEPROM propery's value in the device.
 *  
 * @param dev Device to write the data to.
 * @param name Then name of the property.
 * @param value New value of the property.
 * @param right_display Whether to access the right or left's display property.
 *
 * @returns Z800_TRUE on success.
 * @returns Z800_FALSE on failure.
 * 
 * @note Currently not implemented.
 */
Z800_EXPORT z800_bool z800_write_property(
	z800_device *dev, 
	const char *name, 
	int value, 
	z800_bool right_display
);

#ifndef DOXYGEN

/* Internal EEPROM functions for z800test */

Z800_EXPORT z800_bool z800_dump_eeprom(
	z800_device *dev
);

Z800_EXPORT z800_bool z800_read_eeprom_raw(
	z800_device *dev,
	int address, 
	int *value
);

Z800_EXPORT z800_bool z800_write_eeprom_raw(
	z800_device *dev, 
	int address, 
	int value
);

#endif

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LIBZ800_H */
