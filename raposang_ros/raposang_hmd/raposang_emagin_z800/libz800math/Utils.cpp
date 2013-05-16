#include "Matrix.h"

#include "Utils.h"

bool equivalent(double a, double b, double e)
{
	return fabs(a - b) < e;
}

float clampUnity(float x)
{
	if (x > +1.0f) return +1.0f;
	if (x < -1.0f) return -1.0f;
	return x;
}

Vec3 convertQuatToTaitBryan(const Quat &q)
{
	/**
	 * This routine was written for the OSG axis system...
	 */
	Matrix rotation = Matrix::rotate(Z_AXIS, Y_AXIS) *
			  Matrix::rotate(q) *
			  Matrix::rotate(Y_AXIS, Z_AXIS);
	Matrix mat;

	Vec3 col1(rotation(0, 0), rotation(0, 1), rotation(0, 2));
	double s = col1.length();

	const double magic_epsilon = 0.00001;
	if ( s <= magic_epsilon )
	{
		return Vec3(0.0f, 0.0f, 0.0f);
	}

	Vec3 hpr;

	double oneOverS = 1.0f / s;
	for( int i = 0; i < 3; i++ )
		for( int j = 0; j < 3; j++ )
			mat(i, j) = rotation(i, j) * oneOverS;


	double sin_pitch = clampUnity(mat(1, 2));
	double pitch = asin(sin_pitch);
	hpr[1] = (pitch);

	double cp = cos(pitch);

	if ( cp > -magic_epsilon && cp < magic_epsilon )
	{
		double cr = clampUnity(-mat(2,1));
		double sr = clampUnity(mat(0,1));

		hpr[0] = 0.0f;
		hpr[2] = (atan2(sr,cr));
	}
	else
	{
		double one_over_cp = 1.0 / cp ;
		double sr = clampUnity(-mat(0,2) * one_over_cp);
		double cr = clampUnity(mat(2,2) * one_over_cp);
		double sh = clampUnity(-mat(1,0) * one_over_cp);
		double ch = clampUnity(mat(1,1) * one_over_cp);

		if ( ( equivalent(sh,0.0,magic_epsilon) && equivalent(ch,0.0,magic_epsilon) ) ||
		     ( equivalent(sr,0.0,magic_epsilon) && equivalent(cr,0.0,magic_epsilon) ) )
		{
			cr = clampUnity(-mat(2,1));
			sr = clampUnity(mat(0,1));;

			hpr[0] = 0.0f;
		}
		else
		{
			hpr[0] = -(atan2(sh, ch));
		}

		hpr[2] = (atan2(sr, cr));
	}

	return Vec3(hpr.y(), hpr.x(), hpr.z());
}

Quat convertTaitBryanToQuat(const Vec3 &xyz)
{
	Matrix rotation;

	Vec3 hpr = Vec3(xyz.y(), xyz.x(), xyz.z());

	double ch, sh, cp, sp, cr, sr, srsp, crsp, srcp ;

	const Vec3::value_type magic_epsilon = Vec3::value_type(0.00001);

	if(equivalent(-hpr[0], Vec3::value_type(0.0), magic_epsilon))
	{
		ch = 1.0;
		sh = 0.0;
	}
	else
	{
		sh = sinf((-hpr[0]));
		ch = cosf((-hpr[0]));
	}

	if(equivalent(hpr[1], Vec3::value_type(0.0), magic_epsilon))
	{
		cp = 1.0;
		sp = 0.0;
	}
	else
	{
		sp = sinf((hpr[1]));
		cp = cosf((hpr[1]));
	}

	if(equivalent(hpr[2], Vec3::value_type(0.0), magic_epsilon))
	{
		cr   = 1.0;
		sr   = 0.0;
		srsp = 0.0;
		srcp = 0.0;
		crsp = sp;
	}
	else
	{
		sr   = sinf((hpr[2]));
		cr   = cosf((hpr[2]));
		srsp = sr * sp;
		crsp = cr * sp;
		srcp = sr * cp;
	}

	rotation(0, 0) = (ch * cr - sh * srsp);
	rotation(1, 0) = (-sh * cp);
	rotation(2, 0) = (sr * ch + sh * crsp);

	rotation(0, 1) = (cr * sh + srsp * ch);
	rotation(1, 1) = (ch * cp);
	rotation(2, 1) = (sr * sh - crsp * ch);

	rotation(0, 2) = (-srcp);
	rotation(1, 2) = (sp);
	rotation(2, 2) = (cr * cp);

	rotation(3, 0) = 0.0;
	rotation(3, 1) = 0.0;
	rotation(3, 2) = 0.0;

	rotation(0, 3) = 0.0;
	rotation(1, 3) = 0.0;
	rotation(2, 3) = 0.0;
	rotation(3, 3) = 1.0;

	return (Matrix::rotate(Y_AXIS, Z_AXIS) *
		rotation *
		Matrix::rotate(Z_AXIS, Y_AXIS)).getRotate();
}
