#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <functional>
#include <numeric>
#include <limits>

#include "Matrix.h"
#include "Vec3.h"
#include "Device.h"
#include "Calc.h"
#include "Utils.h"

float cutoff(float v, float cutoff)
{
	if(fabs(v) < cutoff)
	{
		return 0;
	}
	else
	{
		return v;
	}
}

inline Vec3f componentMultiply(const Vec3f& lhs, const Vec3f& rhs)
{ 
    return Vec3f(lhs[0]*rhs[0], lhs[1]*rhs[1], lhs[2]*rhs[2]);
} 

double Qval[4*4] = { 
	0.0006,	0,		0,		0,
	0,		0.0006,	0,		0,
	0,		0,		0.0006,	0,
	0,		0,		0,		0.0006
};

double Rval[4*4] = { 
	3,		0,		0,		0,
	0,		3,		0,		0,	
	0,		0,		3,		0,	
	0,		0,		0,		3
};

Calc::Calc()
: _acc(0.0, -1.0, 0.0)
, _qkf(QKE::Matrix<4,4>(Qval), QKE::Matrix<4,4>(Rval))
{
}

static const double Pi = 3.14159265358979323846264338327950288419717;


bool Calc::updateTrackerData(double dt, const Vec3f &gyr, const Vec3f &acc, const Vec3f &mag)
{
	_gyr = gyr;

	_gyroSpeed.x() = cutoff((gyr.x() * _gyrGain.x()) + _gyrOffset.x(), 0.05);
	_gyroSpeed.y() = cutoff((gyr.y() * _gyrGain.y()) + _gyrOffset.y(), 0.05);
	_gyroSpeed.z() = cutoff((gyr.z() * _gyrGain.z()) + _gyrOffset.z(), 0.05);

	_vecQue.push_back(Vec3d(componentMultiply(gyr, _gyrGain)));
	_gyrOffset = - std::accumulate (_vecQue.begin(), _vecQue.end(), Vec3d(), std::plus<Vec3d>() ) / _vecQue.size();
	if (_vecQue.size() > 60 * 60)
	{	
		_vecQue.pop_front();
	}

	_accQue.push_back(Vec3d(componentMultiply(acc + _accOffset, _accGain)));
	_acc = std::accumulate (_accQue.begin(), _accQue.end(), Vec3d(), std::plus<Vec3d>() ) / _accQue.size();
	_acc.normalize();
	if (_accQue.size() > 5)
	{
		_accQue.pop_front();
	}

	_magQue.push_back(componentMultiply((mag + _magOffset), _magGain));
	if (_magQue.size() > 4)
	{
		_magQue.pop_front();

		float inf = std::numeric_limits<float>::infinity();
		Vec3d maxMag(-inf, -inf, -inf);
		Vec3d minMag(+inf, +inf, +inf);
		for(std::deque<Vec3d>::iterator it = _magQue.begin(); it != _magQue.end();
			++it)
		{
			maxMag.x() = std::max(it->x(), maxMag.x());
			maxMag.y() = std::max(it->y(), maxMag.y());
			maxMag.z() = std::max(it->z(), maxMag.z());
			minMag.x() = std::min(it->x(), minMag.x());
			minMag.y() = std::min(it->y(), minMag.y());
			minMag.z() = std::min(it->z(), minMag.z());
		}

		_mag.x() = (maxMag.x() + minMag.x()) / 2;
		_mag.y() = (maxMag.y() + minMag.y()) / 2;
		_mag.z() = (maxMag.z() + minMag.z()) / 2;
	}	
	else
	{
		_mag = mag;
		_mag = componentMultiply((mag + _magOffset), _magGain);
	}

	_mag.normalize();

	Vec3 d = _acc;
	Vec3 e = d ^ _mag;
	Vec3 n = e ^ d;

	_correctionQue.push_back(acos(n*_mag));
	double correctionAngle = 0.0;
	correctionAngle = std::accumulate(_correctionQue.begin(), _correctionQue.end(), 0.0 ) / _correctionQue.size();
	if (_correctionQue.size() > 10 * 60)
	{
		_correctionQue.pop_front();
	}

#if 0
	printf("%.2f\n", correctionAngle / Pi * 180);
	std::cout.flush();
#endif

	d.normalize();
	e.normalize();
	n.normalize();

	_stabilization = Matrixf(+e.x(), +e.y(), +e.z(), 0.0,
				             -d.x(), -d.y(), -d.z(), 0.0,
				             -n.x(), -n.y(), -n.z(), 0.0,
				              0.0,    0.0,    0.0,   1.0).getRotate().inverse();


	double gv[3] = { _gyroSpeed.x(), _gyroSpeed.y(), _gyroSpeed.z() };
	QKE::Vector<3> g(gv);
	_qkf.predict(dt, gv);

	double qmv[4] = { _stabilization.x(), _stabilization.y(), _stabilization.z(), _stabilization.w() };
	QKE::Quat qm(qmv);

	QKE::Quat q = _qkf.update(qm);

	_output = Quat(q.x(), q.y(), q.z(), q.w());
	return true;
}
	
void Calc::getCalibratedTrackerData(Vec3f &gyr, Vec3f &acc, Vec3f &mag)
{
	acc = _acc;
	mag = _mag;
}

void Calc::getGyroSpeed(Vec3f &gyroSpeed) const
{
	gyroSpeed = _gyroSpeed;
}

void Calc::getStabilization(Quat &quat) const
{
	quat = _stabilization;
}

void Calc::getOrientation(Quat &quat) const
{
	quat = _output;
}

void Calc::setCalibrationData(
	const Vec3f &accOffset,
	const Vec3f &accGain,
	const Vec3f &magOffset,
	const Vec3f &magGain,
	const Vec3f &gyrOffset,
	const Vec3f &gyrGain
)
{
	_accOffset = accOffset;
	_accGain = accGain;
	_magOffset = magOffset;
	_magGain = magGain;
	_gyrOffset = gyrOffset;
	_gyrGain = gyrGain;
}

void Calc::getCalibrationData(
	Vec3f &accOffset,
	Vec3f &accGain,
	Vec3f &magOffset,
	Vec3f &magGain,
	Vec3f &gyrOffset,
	Vec3f &gyrGain
) const
{
	accOffset = _accOffset;
	accGain = _accGain;
	magOffset = _magOffset;
	magGain = _magGain;
	gyrOffset = _gyrOffset;
	gyrGain = _gyrGain;
}



