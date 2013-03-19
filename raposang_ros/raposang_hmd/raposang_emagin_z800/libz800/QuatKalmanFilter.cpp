#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <limits>

#include "QuatKalmanFilter.h"
using namespace QKE;

QuatKalmanFilter::QuatKalmanFilter(
	const Matrix<4,4> &Q,
	const Matrix<4,4> &R
)
: _Q(Q)
, _R(R)
{
	_Pm = Matrix<4,4>::identity();
}

QuatKalmanFilter::~QuatKalmanFilter()
{
}

void QuatKalmanFilter::predict(float dt, const Vector<3> &gyroSpeed)
{
	double t[4] = { gyroSpeed.x(), gyroSpeed.y(), gyroSpeed.z(), 0.0 };
	QKE::Quat gyroQuat=Quat(t);

	/* Predict new orientation */
	_nm = _n + ((gyroQuat * _n) * 0.5f * dt);
	_nm.normalize();

	/* Predict new covariance */
	_Pm = _P + _Q;
}

Quat QuatKalmanFilter::update(const Quat &measurement)
{
	Quat m = measurement;

	if((m.x() * _n.x()) + 
	   (m.y() * _n.y()) +
	   (m.z() * _n.z()) +
	   (m.w() * _n.w()) < 0)
	{
		m.x() = -m.x();
		m.y() = -m.y();
		m.z() = -m.z();
		m.w() = -m.w();
	}

	Matrix<4,4> K = _Pm * (_Pm + _R).inverse();

	_n = _nm + K * (m - _nm);
	_n.normalize();

	_P = (Matrix<4,4>::identity() - K) * _Pm;

	return _n;
}