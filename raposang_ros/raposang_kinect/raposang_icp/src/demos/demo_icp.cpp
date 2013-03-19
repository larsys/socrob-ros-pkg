/*
 *  demo_icp.cpp
 *  
 *  Created by Pedro Vieira on 15/11/11.
 *  
 */

#include <iostream>
#include <string>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"

#include "ICPBlock.h"
#include "ICP_plane2plane.h"
#include "ICPInitializationBlock.h"

#include "UtilitiesBlock.h"


typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class demo{
	
	//thresholds
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
	
	std::string cameras_filename;
	
	ros::NodeHandle n;
	ros::Publisher  pub_pointcloud; //send frames
	ros::Subscriber sub_pointcloud; //get frames
	
	PCloud cloudrec;
		
	Eigen::Matrix4d T;
	
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_positions;

	int acpf;

public:
	demo(std::string _pointcloudtopic,std::string _cameras_filename,double _th,double _prob,double _D,int _max_iterations,double _THRESH_dr,double _THRESH_dt,int _MAXPAIRS,int _MINPAIRS,double _gicp_epsilon,int _k_correspondences){
		T.setIdentity();
		pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("/icp/world", 1);
		sub_pointcloud = n.subscribe(_pointcloudtopic.data(), 1, &demo::get_cloud_Callback, this);
		
		th=_th;
		prob=_prob;
		
		D=_D;
		max_iterations=_max_iterations;
		THRESH_dr=_THRESH_dr;
		THRESH_dt=_THRESH_dt;
		MAXPAIRS=_MAXPAIRS;
		MINPAIRS=_MINPAIRS;
		
		gicp_epsilon=_gicp_epsilon;
		k_correspondences=_k_correspondences;
		
		cameras_filename=_cameras_filename;
		
		acpf = 4;
	}
	
	void makemap_RGB();
	void makemap_RGB_ICP();
	void makemap_RGB_ICP_pl2pl2();
	void makemap_RGB_ICP_pl2pl2_vol();
	

private:	
	void get_cloud_Callback(const PCloud& pcloud){cloudrec=pcloud;}
	
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

void demo::makemap_RGB(){
	
	ros::Rate r(2); // 10 hz
	ros::Time start, end;

		
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	
		
	r.sleep();
	ros::spinOnce();
		
	cloud_d = cloudrec;
		
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
	rgbf.setRansacThreshold(th);
	rgbf.setDesiredP(prob);
	
	Vector3d p1(0.0,0.0,0.0);
	Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
	
			
	int i=1, k=1;
		
	float a1;
	int ft;
	while(ros::ok()) {

		ROS_INFO("%d ", i);	
		
		//update icp clouds
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
		}
			
		start = ros::Time::now();
		ft = rgbf.compute();
		end = ros::Time::now();
		a1 = (end-start).toSec();
		if ( ft > acpf ){	
			k=0;
								
			//update transformation
			T = T*rgbf.T;
			
			ROS_INFO("RGB: %.6f seconds  | inliers= %d", a1, ft);	
			
			//update camera position
			p1 = T.block<3,3>(0,0)*camera_positions[0].first + T.block<3,1>(0,3);
			p2 = T.block<3,3>(0,0)*camera_positions[0].second + T.block<3,1>(0,3);
			camera_positions.push_back(std::make_pair(p1, p2));	
				
			ApplyRTto(cloud_d);
			pub_pointcloud.publish(cloud_d);
						
		}else{
			k=1;
			ROS_INFO("Lost!! ");	
		}
		
		r.sleep();	
		ros::spinOnce();
			
		cloud_d = cloudrec;
			
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(), camera_positions);
	
}

void demo::makemap_RGB_ICP(){
	
	ros::Rate r(2); // 10 hz
	ros::Time start, end;
	
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	


	r.sleep();
	ros::spinOnce();
		
	cloud_d = cloudrec;
		
	ICP icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
	rgbf.setRansacThreshold(th);
	rgbf.setDesiredP(prob);
	
	icp.setRThreshold  (THRESH_dr);
	icp.setTThreshold  (THRESH_dt);		
	icp.setDisThreshold(D);		
	icp.setNiterations (max_iterations);
	icp.setMaxPairs    (MAXPAIRS);
	icp.setMinPairs    (MINPAIRS);	
	
	Vector3d p1(0.0,0.0,0.0);
	Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
		
	int i=1, k=1;
		
	float a1,a2;
	int ft;
	while(ros::ok()) {

		ROS_INFO("%d ", i);	
		
		//update icp clouds
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
			icp.setNewICPinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
			icp.setDcloud(cloud_d);
		}
			
		start = ros::Time::now();
		ft = rgbf.compute();
		end = ros::Time::now();
		a1 = (end-start).toSec();
		if ( ft > acpf ){	
			k=0;
				
			start = ros::Time::now();	
			icp.align(rgbf.T,rgbf.getInliers()); 
			end = ros::Time::now();
			a2 = (end-start).toSec();
					
			//update transformation
			T = T*icp.returnTr();
	
			ROS_INFO("RGB: %.6f seconds  |   ICP:  %.6f seconds   |  inliears= %d", a1, a2,ft);		
			
			//update camera position
			p1 = T.block<3,3>(0,0)*camera_positions[0].first + T.block<3,1>(0,3);
			p2 = T.block<3,3>(0,0)*camera_positions[0].second + T.block<3,1>(0,3);
			camera_positions.push_back(std::make_pair(p1, p2));	
				
			ApplyRTto(cloud_d);
			pub_pointcloud.publish(cloud_d);
						
		}else{
			k=1;
			ROS_INFO("Lost!! ");	
		}
		
		r.sleep();	
		ros::spinOnce();
			
		cloud_d = cloudrec;
			
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(), camera_positions);
	
}


void demo::makemap_RGB_ICP_pl2pl2(){
	ros::Rate r(2); // 10 hz
	ros::Time start, end;
	
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	r.sleep();
	ros::spinOnce();
		
	cloud_d = cloudrec;
		
	ICP_plane2plane icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
	rgbf.setRansacThreshold(th);
	rgbf.setDesiredP(prob);
	
	icp.setRThreshold  (THRESH_dr);
	icp.setTThreshold  (THRESH_dt);		
	icp.setDisThreshold(D);		
	icp.setNiterations (max_iterations);
	icp.setMaxPairs    (MAXPAIRS);
	icp.setMinPairs    (MINPAIRS);	
	
	icp.setGicpEpsilon(gicp_epsilon);
	icp.setKcorrespondences(k_correspondences);
	
	Vector3d p1(0.0,0.0,0.0);
	Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
		
	int i=1, k=1;
		
	float a1,a2;
	int ft;
	while(ros::ok()) {

		ROS_INFO("%d ", i);	
		
		//update icp clouds
		if(k==0){
			rgbf.setNewinputCloud(cloud_d);
			icp.setNewICPinputCloud(cloud_d);
		}else{
			rgbf.setDcloud(cloud_d);
			icp.setDcloud(cloud_d);
		}
			
		start = ros::Time::now();
		ft = rgbf.compute();
		end = ros::Time::now();
		a1 = (end-start).toSec();
		if ( ft > acpf ){	
			k=0;
				
			start = ros::Time::now();	
			icp.align(rgbf.T,rgbf.getInliers()); 
			end = ros::Time::now();
			a2 = (end-start).toSec();
					
			//update transformation
			T = T*icp.returnTr();
	
			ROS_INFO("RGB: %.6f seconds  |   ICP:  %.6f seconds   |  inliears= %d", a1, a2,ft);		
			
			//update camera position
			p1 = T.block<3,3>(0,0)*camera_positions[0].first + T.block<3,1>(0,3);
			p2 = T.block<3,3>(0,0)*camera_positions[0].second + T.block<3,1>(0,3);
			camera_positions.push_back(std::make_pair(p1, p2));	
				
			ApplyRTto(cloud_d);
			pub_pointcloud.publish(cloud_d);
						
		}else{
			k=1;
			ROS_INFO("Lost!! ");	
		}
		
		r.sleep();	
		ros::spinOnce();
			
		cloud_d = cloudrec;
			
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(), camera_positions);
	
}

void demo::makemap_RGB_ICP_pl2pl2_vol(){
	ros::Rate r(2); // 10 hz
	ros::Time start, end;
	
	Eigen::Matrix4d T_backup;
	T_backup.setIdentity();
	
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	r.sleep();
	ros::spinOnce();
		
	cloud_d = cloudrec;
		
	ICP_plane2plane icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
	rgbf.setRansacThreshold(th);
	rgbf.setDesiredP(prob);
	
	icp.setRThreshold  (THRESH_dr);
	icp.setTThreshold  (THRESH_dt);		
	icp.setDisThreshold(D);		
	icp.setNiterations (max_iterations);
	icp.setMaxPairs    (MAXPAIRS);
	icp.setMinPairs    (MINPAIRS);	
	
	icp.setGicpEpsilon(gicp_epsilon);
	icp.setKcorrespondences(k_correspondences);
	
	Vector3d p1(0.0,0.0,0.0);
	Vector3d p2(0.0,0.0,0.1);
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
		if ( ft > acpf ){	
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
				
		ApplyRTto(cloud_d);
		pub_pointcloud.publish(cloud_d);		
		
	
		r.sleep();	
		ros::spinOnce();
			
		cloud_d = cloudrec;
		
		//update icp clouds
		rgbf.setNewinputCloud(cloud_d);
		icp.setNewICPinputCloud(cloud_d);
			
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(), camera_positions);
	
}


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "demo_icp");
	ros::NodeHandle nh("~");	
		
	int sel;
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

	nh.param("sel",sel,1);
	
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
	nh.param<std::string>("cameras_filename",cameras_filename,"cameras.ply");
	
	demo d(pointcloudtopic,cameras_filename,th,prob, D,max_iterations,THRESH_dr,THRESH_dt,MAXPAIRS,MINPAIRS,gicp_epsilon,k_correspondences);	
		
	ROS_INFO("start!!");
	
	switch(sel){
		case 0: {d.makemap_RGB();} break;					//only RGB
		case 1: {d.makemap_RGB_ICP();} break;				//RGB + ICP
		case 2: {d.makemap_RGB_ICP_pl2pl2();} break;		//RGB + ICP plane 2 plane
		case 3: {d.makemap_RGB_ICP_pl2pl2_vol();} break;	//RGB + ICP plane 2 plane velocities
	}
	
	ROS_INFO("done!!");
	
	return 0;
}
