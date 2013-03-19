/*
 *  ICPBlock.h
 *  
 *  Created by Pedro Vieira on 10/28/11.
 *  
 */
 
#ifndef ICPBLOCK_H
#define ICPBLOCK_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>



typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;
typedef std::vector<std::pair<int,int> > Pair;

//The default is the Point-to-Point error metric 
class ICP {
protected:
	float dmax_c;
	float Dmax;
	float D;
	int max_iterations; //max icp loop iterations
	float THRESH_dr;	//Rotation threshold 
	float THRESH_dt;    //translation threshold
	int MAXPAIRS;
	int MINPAIRS;
	
	PCloud cloud_m; //source/model/previously cloud
	PCloud cloud_d; //data/new cloud
	
	std::vector<int> model_indices;
	std::vector<int> data_indices;

	pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree_m;
	
	Eigen::Matrix4d T;
	
	int iter;
	
	//check selected points from cloud_d
	std::vector<int> ind;

	
public:
//constructors
	ICP(){
		cloud_m = PCloud (new Cloud());
		cloud_d = PCloud (new Cloud());
		initvariables();
	}
	ICP(PCloud cl_m, PCloud cl_d){
		cloud_m=cl_m;
		cloud_d=cl_d;
		initvariables();
	}
   ~ICP(){}


	virtual bool align(Eigen::Matrix4d Tini = Eigen::MatrixXd::Identity(4,4), Pair init_f =Pair(0)); 
	
	
	inline void setRThreshold  (const float& thr_r){THRESH_dr=thr_r*3.14159265/180.0;}		
	inline void setTThreshold  (const float& thr_t){THRESH_dt=thr_t;}		
	inline void setDisThreshold(const float& dist) {D=dist; Dmax = D*dmax_c;}		
	inline void setDismaxConst (const float& c)    {dmax_c=c; Dmax = D*dmax_c;}		
	inline void setNiterations (const int& n)      {max_iterations=n;}	
	inline void setMaxPairs    (const int& maxp)   {MAXPAIRS=maxp;}	
	inline void setMinPairs    (const int& minp)   {MINPAIRS=minp;}	
	
	inline void setMcloud(const PCloud& cl_m){cloud_m=cl_m;}			
	inline void setDcloud(const PCloud& cl_d){cloud_d=cl_d;}			
	inline void setNewICPinputCloud(const PCloud& cl_d){cloud_m=cloud_d; cloud_d=cl_d;}
	
	inline Eigen::Matrix3d returnR(){return T.topLeftCorner<3,3> ();}	
	inline Eigen::Vector3d returnT(){return T.topRightCorner<3,1>();}
	inline Eigen::Matrix4d returnTr(){return T;}
	
	inline int returnIter(){return iter;}
	

protected:
	virtual void initvariables();
	
	virtual bool select_macth();
	virtual bool select_macth_useAllpoints();			
	virtual void UpdateAndReject(Pair& init_f);
	virtual void minimize(const Pair& init_f);
	virtual bool error(Eigen::VectorXd xopt_ant , Eigen::VectorXd xopt);
	
	virtual void UpdateXopt(Eigen::VectorXd &xopt, const Eigen::Matrix4d& Tr);
		
};


#include "imp/ICPBlock.hpp"
#endif
