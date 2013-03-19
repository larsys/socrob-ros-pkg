/*
 *  demo_icp_keyframes.cpp
 *  
 *  Created by Pedro Vieira on 01/11/12. (m,d,y)
 *  
 */

#include <iostream>
#include <string>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"

#include "ICPBlock.h"
#include "ICPInitializationBlock.h"

#include "UtilitiesBlock.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class demo{
	
	//thresholds
	int f_th;
	
	int key_th;
	double th;
	double prob;
	double D;
	int max_iterations;
	double THRESH_dr;
	double THRESH_dt;
	int MAXPAIRS;
	int MINPAIRS;
	double gicp_epsilon;
	int k_correspondences;
		
		
	ros::NodeHandle n;
	ros::Publisher  pub_pointcloud; //send frames
	ros::Subscriber sub_pointcloud; //get frames
	
	std::string cameras_filename;
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_positions;
	
		
	PCloud cloudrec;
	
	//for key frames
	PCloud cloud_key;
	RGBFeaturesInit rgbf_key;
		
	Eigen::Matrix4d T;

	bool hframe;

public:
	demo(std::string _pointcloudtopic,std::string _cameras_filename){
		T.setIdentity();
		pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("/icp/world", 1);
		sub_pointcloud = n.subscribe(_pointcloudtopic.data(), 1, &demo::get_cloud_Callback, this);
		
		cameras_filename=_cameras_filename;
		f_th=4;
		
		cloud_key = PCloud(new Cloud());
	}
	
	void makemap_ICP();
	
	
	PCloud getframe(){
		hframe=true;
		ros::Rate r(2); // 10 hz
		
		while(hframe && ros::ok()){
			r.sleep();
			ros::spinOnce();
		}
		
		return cloudrec;
	}
	
	inline void setkeyTh(const int& t) {key_th = t;}					//demo
	inline void setRansacThreshold(const double& t) {th = t;}			//init
	inline void setDesiredP(const double& p) {prob = p;}				//init
	inline void setRThreshold  (const float& thr_r){THRESH_dr=thr_r;}	//p2p
	inline void setTThreshold  (const float& thr_t){THRESH_dt=thr_t;}	//p2p	
	inline void setDisThreshold(const float& dist) {D=dist;}				//p2p
	inline void setNiterations (const int& n)      {max_iterations=n;}	//p2p
	inline void setMaxPairs    (const int& maxp)   {MAXPAIRS=maxp;}		//p2p
	inline void setMinPairs    (const int& minp)   {MINPAIRS=minp;}		//p2p
	inline void setGicpEpsilon(double gicp_e){gicp_epsilon=gicp_e;}		//pl2pl
	inline void setKcorrespondences(int k){k_correspondences=k;}		//pl2pl


private:	
	void get_cloud_Callback(const PCloud& pcloud){cloudrec=pcloud;hframe=false;}
	
	bool checkIfIsNewKeyframe(const PCloud& cloud_new){	
		rgbf_key.setScloud(cloud_key);
		rgbf_key.setDcloud(cloud_new);
	

		int fff = rgbf_key.compute();
	
		ROS_INFO("featuressssss = %d",fff);
		
		if(fff > key_th){
			return false;
		}else{
			*cloud_key=*cloud_new;
			return true;
		}
	}
	
	
	inline void ApplyRTto(PCloud &pc){
	
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		PCloud pcloud = PCloud(new Cloud());
	
		pcloud->header.stamp = pc->header.stamp;
		pcloud->header.frame_id = pc->header.frame_id;
		pcloud->is_dense = pc->is_dense;
	
		pcloud->width = pc->width;
		pcloud->height = pc->height;	
	
		pcloud->points.resize(pcloud->width*pcloud->height);		
		
		for(unsigned int v=0;v<pc->height;v++) {	 //480
			for(unsigned int u=0;u<pc->width;u++) {  //640
				if (!isnan((*pc)(u,v).z)){
					(*pcloud)(u,v).x = (*pc)(u,v).x*T(0,0) + (*pc)(u,v).y*T(0,1) + (*pc)(u,v).z*T(0,2) + T(0,3);
					(*pcloud)(u,v).y = (*pc)(u,v).x*T(1,0) + (*pc)(u,v).y*T(1,1) + (*pc)(u,v).z*T(1,2) + T(1,3);
					(*pcloud)(u,v).z = (*pc)(u,v).x*T(2,0) + (*pc)(u,v).y*T(2,1) + (*pc)(u,v).z*T(2,2) + T(2,3);	
				}else{
					(*pcloud)(u,v).x = bad_point;
					(*pcloud)(u,v).y = bad_point;
					(*pcloud)(u,v).z = bad_point;	
				}
				(*pcloud)(u,v).r = (*pc)(u,v).r;
				(*pcloud)(u,v).g = (*pc)(u,v).g;
				(*pcloud)(u,v).b = (*pc)(u,v).b;
			}
		}
	
		pc = pcloud;
	}

	Eigen::Matrix4d ini_matrix(const Eigen::Vector3d& n){
		Eigen::Matrix4d init = Eigen::MatrixXd::Identity(4,4);
		init(0,3) = n(0);
		init(1,3) = n(1);
		init(2,3) = n(2);
		
		return init;
	}

};

void demo::makemap_ICP(){
	
	ros::Time start, end;
	
	Eigen::Matrix4d T_backup;
	T_backup.setIdentity();
	
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	cloud_s = getframe();
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	*cloud_key = *cloud_s;
		
	cloud_d = getframe();
	
		
	ICP icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
	
	rgbf_key.setRansacThreshold(th);
	rgbf_key.setDesiredP(prob);	
	rgbf.setRansacThreshold(th);
	rgbf.setDesiredP(prob);
	
	icp.setRThreshold  (THRESH_dr);
	icp.setTThreshold  (THRESH_dt);		
	icp.setDisThreshold(D);		
	icp.setNiterations (max_iterations);
	icp.setMaxPairs    (MAXPAIRS);
	icp.setMinPairs    (MINPAIRS);	
	
	Eigen::Vector3d p1(0.0,0.0,0.0);
	Eigen::Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
		
	int i=1;
		
	float a1,a2;
	int ft;
	while(ros::ok()) {

		ROS_INFO("%d ", i);	
					
		start = ros::Time::now();
		ft = rgbf.compute();
		end = ros::Time::now();
		a1 = (end-start).toSec();
		
		start = ros::Time::now();
		if ( ft > f_th ){	
			icp.align(rgbf.T,rgbf.getInliers()); 
		}else{
			icp.align(T_backup); 
		}
		end = ros::Time::now();
		a2 = (end-start).toSec();
					
		//update transformation
		T_backup=icp.returnTr();
		T = T*icp.returnTr();
	
		ROS_INFO("RGB: %.6f seconds  |   ICP:  %.6f seconds   |  inliears= %d", a1, a2,ft);		
			
		//update camera position
		p1 = T.block<3,3>(0,0)*camera_positions[0].first + T.block<3,1>(0,3);
		p2 = T.block<3,3>(0,0)*camera_positions[0].second + T.block<3,1>(0,3);
		camera_positions.push_back(std::make_pair(p1, p2));	
		
		
		if(checkIfIsNewKeyframe(cloud_d)){
			ApplyRTto(cloud_d);
			pub_pointcloud.publish(cloud_d);
		}	
						
		cloud_d = getframe();
		
		
		//update icp clouds
		rgbf.setNewinputCloud(cloud_d);
		icp.setNewICPinputCloud(cloud_d);
			
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(), camera_positions);
	
}



int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "demo_icp_keyframes");
	ros::NodeHandle nh("~");	

	int sel;
	
	int key_th;
	double th;
	double prob;
	double D;
	int max_iterations;
	double THRESH_dr;
	double THRESH_dt;
	int MAXPAIRS;
	int MINPAIRS;
	double gicp_epsilon;
	int k_correspondences;
	
	std::string pointcloudtopic;
	std::string cameras_filename;

	nh.param("sel",sel,0);
	nh.param("key_th",key_th,4);
	
	nh.param("th",th,0.0001);
	nh.param("prob",prob,0.999);
	
	nh.param("D",D,0.005);
	nh.param("max_iterations",max_iterations,100);
	nh.param("THRESH_dr",THRESH_dr,0.5);
	nh.param("THRESH_dt",THRESH_dt,0.002); //0.004 fo pl2pl
	nh.param("MAXPAIRS",MAXPAIRS,2000);
	nh.param("MINPAIRS",MINPAIRS,200);
	
	nh.param("gicp_epsilon",gicp_epsilon,0.0004);
	nh.param("k_correspondences",k_correspondences,20);

	nh.param<std::string>("pointcloudtopic",pointcloudtopic,"/camera/rgb/points");
	nh.param<std::string>("cameras_filename",cameras_filename,"~/Desktop/results/cameras.ply");
	
	demo d(pointcloudtopic,cameras_filename);	
	d.setkeyTh(key_th); 
	d.setRansacThreshold(th);
	d.setDesiredP(prob);
	d.setRThreshold (THRESH_dr);
	d.setTThreshold (THRESH_dt);	
	d.setDisThreshold(D);
	d.setNiterations(max_iterations);
	d.setMaxPairs (MAXPAIRS);
	d.setMinPairs (MINPAIRS);
	d.setGicpEpsilon(gicp_epsilon);
	d.setKcorrespondences(k_correspondences);
		
		
	ROS_INFO("start!!");
	switch(sel){
		case 0: {d.makemap_ICP();} break;					//ICP point to point
	}
	ROS_INFO("done!!");
	
	return 0;
}
