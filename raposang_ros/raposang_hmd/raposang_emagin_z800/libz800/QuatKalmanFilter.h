#pragma once

#include <cassert>
#include <cmath>
#include <limits>

void test();

namespace QKE {

template<unsigned N, unsigned M>
class Matrix
{
public:
	typedef double value_type;

	Matrix()
	{
		for(unsigned i = 0; i < N; ++i)
			for(unsigned j = 0; j < M; ++j)
				_m[i][j] = 0.0;
	}

	Matrix(value_type (&elements)[N * M])
	{
		for(unsigned i = 0; i < N; ++i)
			for(unsigned j = 0; j < M; ++j)
				_m[i][j] = elements[j * N + i];
	}

	static Matrix<N,M> identity()
	{
		Matrix<N,M> m;
		for(unsigned i = 0; i < N; ++i)
			m._m[i][i] = 1.0;
		return m;
	}

	const value_type &operator()(int i, int j) const
	{
		assert(i < N && j < M);
		return _m[i][j];
	}

	value_type &operator()(int i, int j)
	{
		assert(i < N && j < M);
		return _m[i][j];
	}

	template<unsigned L>
	Matrix<L, M> operator *(const Matrix<L, N> &rhs) const
	{
		Matrix<L, M> c;

		for(unsigned i = 0; i < L; ++i)
			for(unsigned j = 0; j < M; ++j)
			{
				value_type t = 0;
				for(unsigned k = 0; k < N; ++k)
				{
					t += _m[k][j] * rhs._m[i][k];
				}
				c._m[i][j] = t;
			}

		return c;
	}


	Matrix<N, M> operator +(const Matrix<N, M> &rhs) const
	{
		Matrix<N, M> c;

		for(unsigned i = 0; i < N; ++i)
			for(unsigned j = 0; j < M; ++j)
			{
				c._m[i][j] = _m[i][j] + rhs._m[i][j];
			}
		return c;
	}

	Matrix<N, M> operator -(const Matrix<N, M> &rhs) const
	{
		Matrix<N, M> c;

		for(unsigned i = 0; i < N; ++i)
			for(unsigned j = 0; j < M; ++j)
			{
				c._m[i][j] = _m[i][j] - rhs._m[i][j];
			}
		return c;
	}

	Matrix<M, N> transposed() const
	{
		Matrix<M, N> c;

		for(unsigned i = 0; i < N; ++i)
			for(unsigned j = 0; j < M; ++j)
			{
				c._m[j][i] = _m[i][j];
			}
		return c;
	}

	bool operator==(const Matrix<N, M> &rhs) const
	{
		const double epsilon = 0.00001;
		for(unsigned i = 0; i < N; ++i)
			for(unsigned j = 0; j < M; ++j)
				if(fabs(_m[i][j] - rhs._m[i][j]) > epsilon)
					return false;
		return true;
	}

	Matrix<N, N> inverse() const
	{
		Matrix<N, N> ret,orig;
		ret = Matrix<N, N>::identity();
		orig = orig + *this;
		value_type tmp;

		for(int j=0; j<N; j++)
		{
			/* Go through all rows */
			assert(fabs(orig._m[j][j]) > std::numeric_limits<double>::epsilon());

			tmp = orig._m[j][j];

			for(int i=j; i<N; i++)
			{
				/* Go diagonal through all columns */
				orig._m[i][j]=orig._m[i][j]/tmp;
			}
			for(int k=0; k<N; k++)
			{
				/* Go through the return-matrix with the same operation */
				ret._m[k][j]=ret._m[k][j]/tmp;
			}

			for(int l=j+1; l<N; l++)
			{
				tmp=orig._m[j][l];
				for (int m=j;m<N;m++){
					orig._m[m][l]=orig._m[m][l]-tmp*orig._m[m][j];
				}
				for (int n=0;n<N;n++){
					ret._m[n][l]=ret._m[n][l]-tmp*ret._m[n][j];
				}

			}

		}

		for (int j=N-1; j>=0; j--)
		{	
			/* Go through all rows backwards */

			for (int l=j-1; l>=0; l--)
			{
				tmp=orig._m[j][l];
				for (int m=j;m<N;m++)
				{
					orig._m[m][l]=orig._m[m][l]-tmp*orig._m[m][j];
				}
				for (int n=0;n<N;n++){
					ret._m[n][l]=ret._m[n][l]-tmp*ret._m[n][j];
				}
			}
		}

		assert((*this * ret == Matrix<N, N>::identity()));

		return ret;
	};


	void dump() const
	{
		printf("----------------------\n");
		for(unsigned j = 0; j < M; ++j)
		{
			for(unsigned i = 0; i < N; ++i)
				printf("%+.8f ", _m[i][j]);
			printf("\n");		
		}
		fflush(stdout);
	}

	value_type _m[N][M];
};

template<unsigned M>
class Vector : public Matrix<1, M>
{
public:
	typedef typename Matrix<1, M>::value_type value_type;

	Vector()
	{
	}

	Vector(value_type (&elements)[1 * M])
	: Matrix<1, M>(elements)
	{
	}

	const Vector<M> &operator=(const Matrix<1, M> &m)
	{
		for(unsigned j = 0; j < M; ++j)
			(*this)(0,j) = m(0,j);
		return *this;
	}


        inline value_type length() const
        {
			value_type l = 0.0;
			for(unsigned j = 0; j < M; ++j)
				l += (*this)(0,j) * (*this)(0,j);
            return sqrt(l);
        }

        /** Normalize the vector so that it has length unity.
          * Returns the previous length of the vector.
        */
        inline value_type normalize()
        {
            value_type norm = length();
            if (norm>0.0)
            {
                value_type inv = 1.0f/norm;
				for(unsigned j = 0; j < M; ++j)
					(*this)(0,j) *= inv;
            }                
            return( norm );
        }


	const value_type &x() const { return (*this)(0, 0); }
	const value_type &y() const { return (*this)(0, 1); }
	const value_type &z() const { return (*this)(0, 2); }
	const value_type &w() const { return (*this)(0, 3); }

	value_type &x() { return (*this)(0, 0); }
	value_type &y() { return (*this)(0, 1); }
	value_type &z() { return (*this)(0, 2); }
	value_type &w() { return (*this)(0, 3); }
};

class Quat : public Vector<4>
{
public:
	Quat()
	{
		_m[0][3] = 1.0;
	}

	Quat(value_type (&elements)[4])
	: Vector<4>(elements)
	{
	}

	const Quat &operator=(const Matrix<1, 4> &m)
	{
		for(unsigned j = 0; j < 4; ++j)
			(*this)(0,j) = m(0,j);
		return *this;
	}

	Quat operator*(double v) const
    {
		double a[4] = { 
			x() * v, y() * v, z() * v, w() * v 
		};

		return Quat(a);
	}

	Quat operator*(const Quat& rhs) const
    {
		double a[4] = {
					rhs.w()*x() + rhs.x()*w() + rhs.y()*z() - rhs.z()*y(),
                    rhs.w()*y() - rhs.x()*z() + rhs.y()*w() + rhs.z()*x(),
                    rhs.w()*z() + rhs.x()*y() - rhs.y()*x() + rhs.z()*w(),
                    rhs.w()*w() - rhs.x()*x() - rhs.y()*y() - rhs.z()*z()
		};
		
		return Quat(a);
	}
};

class QuatKalmanFilter
{
public:
	QuatKalmanFilter(
		const Matrix<4,4> &Q,
		const Matrix<4,4> &R
	);

	virtual ~QuatKalmanFilter();

	void predict(float dt, const Vector<3> &gyroSpeed);
	Quat update(const Quat &m);

protected:
	/* Input from last iteration */
	Quat _n;
	Matrix<4,4> _P;

	/* Move from predict to update */
	Quat _nm;
	Matrix<4,4> _Pm;
	Matrix<4,4> _K;

	/* Constants */
	const Matrix<4,4> _Q;
	const Matrix<4,4> _R;
};

}
