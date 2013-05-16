/*
 *  surfels_rgbd.h
 *  
 *  Created by Pedro Vieira on 01/17/12.
 *  
 */
 

#ifndef SURFELS_RGBD_H
#define SURFELS_RGBD_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <vector>
#include <Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class SurfelsRGBDModel{
protected:
	float update_max_normal_angle;
    float update_max_dist;
    float mindist;
	
	std::vector<pcl::PointSurfel> m_surfels;
		
		
	//Deapth calibration parameters
	double fx, fy;
	double cx, cy;
	
		
public:	
	SurfelsRGBDModel(float n_angle=60, float max_dist=0.1, float _mindist=0.2){
		update_max_normal_angle = n_angle;
		update_max_dist = max_dist;
		mindist = _mindist;
		setCalibParameters();
	}
	
	bool addNewView(const PCloud &cloud_d, Eigen::Matrix4d T);
	void computeMesh();
	
	void reset() {m_surfels.clear();}
	
	void setCalibParameters(double _cx = 3.1729047051634814e+02,
							double _cy = 2.3906595325409393e+02,
							double _fx = 5.8482385484324675e+02,
							double _fy = 5.8545433983133273e+02){
		fx = _fx;
		fy = _fy; 
		cx = _cx; 
		cy = _cy;
	}

	
protected:
	inline void normalize(Eigen::Vector4d &vector){
		vector.block<3,1>(0,0) = vector.block<3,1>(0,0) / vector.block<3,1>(0,0).norm();
	}
	inline Eigen::Vector4d computeNormal(int ind,pcl::KdTreeFLANN< pcl::PointXYZRGB > *kdtree,const PCloud &cloud){
		std::vector<int> kk_indices(20);
		std::vector<float> kk_distances(20);
	
	
		Eigen::Vector4d normal(0.0,0.0,0.0,1.0);
		Eigen::Vector3d mean(0.0,0.0,0.0);
		Eigen::Matrix3d cov;
		int j;
				
		Eigen::MatrixXd points(3, 20);
			
		kdtree->nearestKSearch  (ind, 20, kk_indices, kk_distances); 
			
		for(j=0; j<kk_indices.size(); j++){
			points(0,j) = cloud->points[kk_indices[j]].x;
			points(1,j) = cloud->points[kk_indices[j]].y;
			points(2,j) = cloud->points[kk_indices[j]].z;
				
			mean = mean + points.block<3,1>(0,j);
		}
			
		if(j!=20) points.conservativeResize(Eigen::NoChange, j);
			
		mean = mean/j;
			
		for (unsigned int k=0; k<j; k++)
			points.block<3,1>(0,k) = points.block<3,1>(0,k) - mean;
			
		cov = points * points.transpose();
				
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
		
		//the eigenvalues are sorted in increasing order
		normal.block<3,1>(0,0) = eigensolver.eigenvectors().col(0);
			
		if (normal(0)*cloud->points[ind].x + normal(1)*cloud->points[ind].y + normal(2)*cloud->points[ind].z > 0) {
			normal(0) = -normal(0);
			normal(1) = -normal(1);
			normal(2) = -normal(2);
		}
		
		return normal;				
	}
	
};


#endif
