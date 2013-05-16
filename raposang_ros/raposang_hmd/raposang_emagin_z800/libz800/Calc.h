#pragma once

/* STL */
#include <Vec3f.h>
#include <Quat.h>
#include <deque>

#include "QuatKalmanFilter.h"

class Device;

class Calc
{
public:
	Calc();

	void setCalibrationData(
		const Vec3f &accOffset,
		const Vec3f &accGain,
		const Vec3f &magOffset,
		const Vec3f &magGain,
		const Vec3f &gyrOffset,
		const Vec3f &gyrGain
	);

	void getCalibrationData(
		Vec3f &accOffset,
		Vec3f &accGain,
		Vec3f &magOffset,
		Vec3f &magGain,
		Vec3f &gyrOffset,
		Vec3f &gyrGain
	) const;

	bool updateTrackerData(double dt, const Vec3f &gyr, const Vec3f &acc, const Vec3f &mag);
	void getCalibratedTrackerData(Vec3f &gyr, Vec3f &acc, Vec3f &mag);
	void getStabilization(Quat &q) const;
	void getGyroSpeed(Vec3f &gyroSpeed) const;
	void getOrientation(Quat &q) const;

protected:
	Vec3f _gyr;
	Vec3f _acc;
	Vec3f _mag;

	QKE::QuatKalmanFilter _qkf;

	Vec3f _accOffset;
	Vec3f _accGain;
	Vec3f _magOffset;
	Vec3f _magGain;
	Vec3f _gyrOffset;
	Vec3f _gyrGain;

	Vec3f _gyroSpeed;

	Quat _q0;
	Quat _q1;
	Quat _q2;
	Quat _output;

	Quat _stabilization;

	std::deque<Vec3d> _vecQue;
	std::deque<Vec3d> _magQue;
	std::deque<Vec3d> _accQue;

	std::deque<double> _correctionQue;
};
