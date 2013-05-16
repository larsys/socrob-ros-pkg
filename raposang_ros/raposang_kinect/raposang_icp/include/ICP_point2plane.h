/*
 *  ICP_point2plane.h
 *  
 *  Created by Pedro Vieira on 11/6/11.
 *  
 */
 
#ifndef ICP_POINT2PLANE_H
#define ICP_POINT2PLANE_H

#include <Eigen/Dense>
#include "ICPBlock.h"



//point to plane
class ICP_p2plane : public ICP {
protected:
	int k_correspondences;

public:
	ICP_p2plane() : ICP(){}
	ICP_p2plane(PCloud cl_s, PCloud cl_d) : ICP(cl_s, cl_d){} 
	
	void setKcorrespondences(int k){k_correspondences=k;}
	
protected:
	void minimize(const Pair& init_f);
	void ComputeNormal(int pind, Eigen::Vector3d &normal);
	
};

#include "imp/ICP_point2plane.hpp"

#endif
