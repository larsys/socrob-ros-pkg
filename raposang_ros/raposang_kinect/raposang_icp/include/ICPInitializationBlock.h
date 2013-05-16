/*
 *  ICPInitializationBlock.h
 *  
 *  Created by Pedro Vieira on 10/23/11.
 *  
 */
 

#ifndef ICPINITIALIZATIONBLOCK_H
#define ICPINITIALIZATIONBLOCK_H


#include <cv.h> 
#include <vector> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>



typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;
typedef std::vector<std::pair<int,int> > Pair;

class RGBFeaturesInit{
protected:
	PCloud cloud_s; //source 
	PCloud cloud_d; //data (new)
	
	double th;    //1cm 
	double prob;  //99,9%

	Pair matches3D;
	Pair inliers;

public:	
	Eigen::Matrix4d T;

	RGBFeaturesInit(){
			th=0.0001; 
			prob=0.999;
			T.setIdentity();
	}
	RGBFeaturesInit(PCloud c_s, PCloud c_d){
		cloud_s = c_s;
		cloud_d = c_d;
		T.setIdentity();	
		th=0.0001; 
		prob=0.999;
	}
	
	int compute();
	int computeSURF();

	void setScloud(const PCloud& c_s) {cloud_s = c_s;}
	void setDcloud(const PCloud& c_d) {cloud_d = c_d;}
	void setNewinputCloud(const PCloud& c_d){cloud_s=cloud_d;cloud_d = c_d;}
	void setRansacThreshold(const double& t) {th = t;}
	void setDesiredP(const double& p) {prob = p;}
	Pair getInliers(){return inliers;}
	
	
protected:
	cv::Mat extractRGBfromCloud(const PCloud& cloud);
	void computeORBfeatures(const cv::Mat& im , cv::Mat &desc , std::vector<cv::KeyPoint> &kp);
	void computeSURFfeatures(const cv::Mat& im , cv::Mat &desc , std::vector<cv::KeyPoint> &kp);
	void macthpoints(const cv::Mat& desc_s,const cv::Mat& desc_d, std::vector<cv::DMatch> &matches);
	void make3Dpairs(std::vector<cv::KeyPoint> kp_s, std::vector<cv::KeyPoint> kp_d, std::vector<cv::DMatch> matches);
	void TakeOutOutliers(); 
	
};


#endif
