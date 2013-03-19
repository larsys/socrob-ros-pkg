#include <stdio.h>
#include <stddef.h>

#include "Device.h"
#include "Calc.h"
#include "QuatKalmanFilter.h"

#include "libz800.h"

typedef struct z800_device
{
	z800_device(int debugLevel)
	: _dev(debugLevel)
	, _connected(false)
	{
	}

	Device _dev;
	Calc _calc;
	z800_calibration_data _calibration_data;
	bool _connected;
	bool _shutdown;
	Vec3f _gyr;
	Vec3f _acc;
	Vec3f _mag;
	Quat _s;
	Quat _q;
} z800_device;

z800_device *z800_connect(int debug_level)
{
	//test();

	z800_device *p = new z800_device(debug_level);

	if(p->_dev.connect() == false)
	{
		delete p;
		return NULL;
	}


	z800_poll_tracker(p);

	float gyr[3], acc[3], mag[3];
	if(z800_get_raw_tracker_data(p, &gyr, &acc, &mag) == 0)
	{
		printf("z800_get_raw_tracker_data() failed\n");
		delete p;
		return NULL;
	}

	z800_calibration_data calibration_data;

	calibration_data.acc_offset[0] = 0.048f;
	calibration_data.acc_offset[1] = 0.006f;
	calibration_data.acc_offset[2] = -0.029f;
	calibration_data.acc_gain[0] = 2.018f;
	calibration_data.acc_gain[1] = 2.002f;
	calibration_data.acc_gain[2] = 2.016f;

	calibration_data.mag_offset[0] = 0.072f;
	calibration_data.mag_offset[1] = 0.003f;
	calibration_data.mag_offset[2] = 0.125f;
	calibration_data.mag_gain[0] = 3.595f;
	calibration_data.mag_gain[1] = 3.297f;
	calibration_data.mag_gain[2] = 2.793f;

	calibration_data.gyr_gain[0] = 16.5f;
	calibration_data.gyr_gain[1] = 16.5f;
	calibration_data.gyr_gain[2] = 16.5f;
	calibration_data.gyr_offset[0] = -gyr[0] * calibration_data.gyr_gain[0];
	calibration_data.gyr_offset[1] = -gyr[1] * calibration_data.gyr_gain[1];
	calibration_data.gyr_offset[2] = -gyr[2] * calibration_data.gyr_gain[2];

	z800_set_calibration_data(p, &calibration_data);

	p->_connected = true;

	return p;
}

void z800_disconnect(z800_device *p)
{	
	if(!p) return;
	p->_dev.disconnect();
	delete p;
}

void z800_set_debug_level(z800_device *p, int debug_level)
{
	if(!p) return;
	p->_dev.setDebugLevel(debug_level);
}

z800_bool z800_wake(z800_device *p)
{
	if(!p) return Z800_FALSE;
	return p->_dev.wake() ? Z800_TRUE : Z800_FALSE;
}

z800_bool z800_set_stereo_mode(z800_device *p, z800_bool enabled)
{
	if(!p) return Z800_FALSE;
	return p->_dev.setStereoMode(enabled != Z800_FALSE) ? Z800_TRUE : Z800_FALSE;
}

z800_bool z800_flip_stereo(z800_device *p)
{
	if(!p) return Z800_FALSE;
	return p->_dev.flipStereo() ? Z800_TRUE : Z800_FALSE;
}

z800_bool z800_step_brightness(z800_device *p)
{
	if(!p) return Z800_FALSE;
	return p->_dev.stepBrightness() ? Z800_TRUE : Z800_FALSE;
}

int z800_get_firmware_version(z800_device *p)
{
	if(!p) return -1;
	return p->_dev.getFirmwareVersion();
}

z800_bool z800_poll_tracker(z800_device *p)
{
	if(!p) return Z800_FALSE;
	Vec3f gyr, acc, mag;
	double dt;

	dt = p->_dev.getTrackerData(gyr, acc, mag);
	if(dt < 0.0)
	{
		return Z800_FALSE;
	}

	if(dt == 0.0)
	{
		return Z800_TRUE;
	}

	p->_gyr = gyr;
	p->_acc = acc;
	p->_mag = mag;
	p->_calc.updateTrackerData(dt, p->_gyr, p->_acc, p->_mag);
	p->_calc.getStabilization(p->_s);
	p->_calc.getOrientation(p->_q);

	return Z800_TRUE;
}

z800_bool z800_get_raw_tracker_data(z800_device *p, float (*gyr)[3], float (*acc)[3], float (*mag)[3])
{
	if(!p) return Z800_FALSE;
	*reinterpret_cast<Vec3f *>(gyr) = p->_gyr;
	*reinterpret_cast<Vec3f *>(acc) = p->_acc;
	*reinterpret_cast<Vec3f *>(mag) = p->_mag;
	return Z800_TRUE;
}


z800_bool z800_get_calibrated_tracker_data(z800_device *p, float (*gyr)[3], float (*acc)[3], float (*mag)[3])
{
	if(!p) return Z800_FALSE;
	*reinterpret_cast<Vec3f *>(gyr) = p->_gyr;
	Vec3f dummy;
	p->_calc.getCalibratedTrackerData(
		dummy,
		*reinterpret_cast<Vec3f *>(acc),
		*reinterpret_cast<Vec3f *>(mag)
	);
	return Z800_TRUE;
}

z800_bool z800_get_stabilization(z800_device *p, double (*q)[4])
{
	if(!p) return Z800_FALSE;
	*reinterpret_cast<Quat *>(q) = p->_s;
	return Z800_TRUE;
}

z800_bool z800_get_orientation(z800_device *p, double (*q)[4])
{
	if(!p) return Z800_FALSE;
	*reinterpret_cast<Quat *>(q) = p->_q;
	return Z800_TRUE;
}

z800_bool z800_set_calibration_data(z800_device *p, const struct z800_calibration_data *calibration_data)
{
	if(!p) return Z800_FALSE;
	p->_calibration_data = *calibration_data;
	p->_calc.setCalibrationData(
		*reinterpret_cast<const Vec3f *>(calibration_data->acc_offset),
		*reinterpret_cast<const Vec3f *>(calibration_data->acc_gain),
		*reinterpret_cast<const Vec3f *>(calibration_data->mag_offset),
		*reinterpret_cast<const Vec3f *>(calibration_data->mag_gain),
		*reinterpret_cast<const Vec3f *>(calibration_data->gyr_offset),
		*reinterpret_cast<const Vec3f *>(calibration_data->gyr_gain)
	);
	return Z800_TRUE;
}

z800_bool z800_get_calibration_data(z800_device *p, struct z800_calibration_data *calibration_data)
{
	if(!p) return Z800_FALSE;
	*calibration_data = p->_calibration_data;
	return Z800_TRUE;
}

z800_bool z800_get_gyro_speed(z800_device *p, float (*gyrSpeed)[3])
{
	if(!p) return Z800_FALSE;
	p->_calc.getGyroSpeed(
		*reinterpret_cast<Vec3f *>(gyrSpeed)
	);
	return Z800_TRUE;
}

const char * const *z800_get_properties(z800_device *p)
{
	if(!p) return NULL;

	return p->_dev.getPropertyNames();
}

z800_bool z800_get_property_meta_info(z800_device *p, const char *name, struct z800_property *out)
{
	if(!p) return Z800_FALSE;

	Property *prop = p->_dev.getProperty(name);
	if(prop == NULL)
	{
		return Z800_FALSE;
	}

	out->stereo = (prop->getRightLowAddr() > -1) ? 1 : 0;
	out->min_value = prop->getMin();
	out->max_value = prop->getMax();
	out->default_value = prop->getDefault();

	return Z800_TRUE;
}

z800_bool z800_read_property(z800_device *p, const char *name, int *value, z800_bool right_display)
{
	if(!p) return Z800_FALSE;
	if(p->_dev.readProperty(name, *value, right_display != Z800_FALSE) == false) return Z800_FALSE;
	return Z800_TRUE;
}

z800_bool z800_write_property(z800_device *p, const char *name, int value, z800_bool right_display)
{
	if(!p) return Z800_FALSE;
	if(p->_dev.writeProperty(name, value, right_display != Z800_FALSE) == false) return Z800_FALSE;
	return Z800_TRUE;
}

z800_bool z800_dump_eeprom(z800_device *p)
{
	if(!p) return Z800_FALSE;
	if(p->_dev.dumpEEPROM() == false) return Z800_FALSE;
	return Z800_TRUE;
}

z800_bool z800_read_eeprom_raw(z800_device *p, int address, int *value)
{
	if(!p) return Z800_FALSE;
	return Z800_FALSE;
}

z800_bool z800_write_eeprom_raw(z800_device *p, int address, int value)
{
	if(!p) return Z800_FALSE;
	return Z800_FALSE;
}
