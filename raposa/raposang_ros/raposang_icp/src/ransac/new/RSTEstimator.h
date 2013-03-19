#ifndef _RSTESTIMATOR_H_
#define _RSTESTIMATOR_H_

#include <iostream>
#include "ParameterEstimator.h"
#include <Eigen/Dense>

/**
 * This class estimates the parameters of an homography .
 *
 */

class RSTEstimator : public ParameterEstimator<std::pair<Eigen::Vector3d,Eigen::Vector3d> ,double> {
public:
	RSTEstimator(double delta);

	/**
	 * Compute the line defined by the given data points.
	 * @param data A vector containing two 2D points.
	 * @param This vector is cleared and then filled with the computed parameters.
	 *        The parameters of the line passing through these points [n_x,n_y,a_x,a_y]
	 *        where ||(n_x,ny)|| = 1.
	 *        If the vector contains less than two points then the resulting parameters
	 *        vector is empty (size = 0).
	 */
	virtual void estimate(std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> *> &data, std::vector<double> &parameters);

	/**
	 * Compute a least squares estimate of the line defined by the given points.
	 * This implementation is of an orthogonal least squares error.
	 *
	 * @param data The line should minimize the least squares error to these points.
	 * @param parameters This vector is cleared and then filled with the computed parameters.
	 *                   Fill this vector with the computed line parameters [n_x,n_y,a_x,a_y]
	 *                   where ||(n_x,ny)|| = 1.
	 *                   If the vector contains less than two points then the resulting parameters
	 *                   vector is empty (size = 0).
	 */
	virtual void leastSquaresEstimate(std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> *> &points, std::vector<double> &parameters);

	/**
	 * Return true if the distance between the line defined by the parameters and the
	 * given point is smaller than 'delta' (see constructor).
	 * @param parameters The line parameters [n_x,n_y,a_x,a_y].
	 * @param data Check that the distance between this point and the line is smaller than 'delta'.
	 */
	virtual bool agree(std::vector<double> &parameters, std::pair<Eigen::Vector3d,Eigen::Vector3d> &data);


private:
	double deltaSquared; //given line L and point P, if dist(L,P)^2 < m_delta^2 then the point is on the line
};

#endif //_LINE_PARAM_ESTIMATOR_H_
