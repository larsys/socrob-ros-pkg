#pragma once

#include <map>

#include <Vec3f.h>

#include "Timer.h"
#include "Property.h"

struct libusb_context;
struct libusb_device_handle;

/**
 * This class provides raw access to the actual hardware of the Z800.
 */
class Device
{
public:
	/**
	 * Constructor.
	 */
	Device(int debugLevel);

	/**
	 * Destructor.
	 */
	virtual ~Device();

	/**
	 * Connects to the device (both the tracker and display).
	 *
	 * @returns true on success.
	 * @returns false on failure.
	 */
	bool connect();

	/**
	 * Disconnects from the device.
	 */
	void disconnect();

	void setDebugLevel(int debugLevel);

	/**
	 * Retrieves the firmware version.
	 *
	 * @returns (major << 8) | minor on success.
	 * @returns -1 on failure.
	 */
	int getFirmwareVersion();

	/**
	 * Sets the stereo mode.
	 *
	 * @param enabled Whether to enable or disable the stereo mode.
	 *
	 * @returns true on success.
	 * @returns false on failure.
	 */
	bool setStereoMode(bool enabled);

	/**
	 * Flips the left and right eye in stereo mode.
	 *
	 * @returns true on success.
	 * @returns false on failure.
	 */
	bool flipStereo();

	/**
	 * Steps the display brightness up one level.
	 *
	 * @returns true on success.
	 * @returns false on failure.
	 */
	bool stepBrightness();

	/**
	 * Awakens the display from standby mode or resets the standby timer.
	 *
	 * @returns true on success.
	 * @returns false on failure.
	 */
	bool wake();

	/**
	 * Polls for actual tracker data.
	 *
	 * @param gyr Reference to array to retrieve HPR values from gyroscope.
	 * @param acc Reference to array to retrieve XYZ vector from accelerometer.
	 * @param mag Reference to array to retrieve XYZ vector from magnetometer.
	 *
	 * @returns the elapsed time since the last call on success.
	 * @returns -1.0 on failure.
	 *
	 * @note All values are reported in the range [-1..1] with unknown units.
	 * @note This method can report failure but still write into some of the passed arrays.
	 */
	double getTrackerData(Vec3f &gyr, Vec3f &acc, Vec3f &mag);

	std::vector<std::string> getProperties();

	Property *getProperty(const std::string &name);

	bool readProperty(const std::string &name, int &value, bool rightDisplay);

	bool writeProperty(const std::string &name, int value, bool rightDisplay);

	const char * const *getPropertyNames();

	bool dumpEEPROM();

	bool readEEPROM(int address, int &value);

	bool writeEEPROM(int address, int value);

protected:

	/**
	 * Perform a read-back of the display, filling _lastCmd and _eepromRead.
	 *
	 * @returns true on success.
	 * @returns false on failure.
	 */
	bool pollDisplay();

	/**
	 * Sends a command to the display, performing hand-shaking to ensure retrieval.
	 */
	bool sendCmd(int verb, int noun);

	/**
	 * Sends a command to the display without ensuring retrieval.
	 */
	bool sendCmdRaw(int verb, int noun);

	/**
	 * Handle of the current libusb context.
	 */
	libusb_context *_ctx;

	/**
	 * Handle to the opened tracker device.
	 */
	libusb_device_handle *_tracker;

	/**
	 * Handle to the opened display device.
	 */
	libusb_device_handle *_display;

	/**
	 * Last command read-back from the display. Set by pollDisplay().
	 */
	int _lastCmd;

	/**
	 * Firmware version byte read-back from the display. Set by pollDisplay().
	 */
	int _eepromRead;

	/**
	 * Timer to obtain the elapsed time since the last time gyro values were fetched.
	 */
	Timer _timer;

	int _debugLevel;

	std::map<std::string, Property> _properties;

	const std::vector<char *> _propertyNames;
};
