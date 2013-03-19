/*
 *  demo_icp_keyframes_folders.cpp
 *  
 *  Created by Pedro Vieira on 01/14/12. (m,d,y)
 *  
 * 	Cria menagens com o point cloud e transformacao, e envia para a class world
 */

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"
#include "raposang_icp/pointcloud_t.h"

#include "ICPBlock.h"
#include "ICPInitializationBlock.h"

#include <Eigen/Dense>



typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class demo{
	
	ICP icp;
	RGBFeaturesInit rgbf;
	
	//threshold
	int f_th;
	int psize_th;
	
	bool hframe;
	int id;
			
	//ROS
	ros::NodeHandle n;
	ros::Publisher  pub_pointcloud; //send frames
	ros::Subscriber sub_pointcloud; //get frames
	
	
	PCloud cloudrec;
		
	Eigen::Matrix4d T;


public:
	demo(std::string _pointcloudtopic){
		T.setIdentity();
		pub_pointcloud = n.advertise< raposang_icp::pointcloud_t > ("/icp/world", 1);
		sub_pointcloud = n.subscribe(_pointcloudtopic.data(), 1, &demo::get_cloud_Callback, this);
		
		id=0;
		f_th=4;
		psize_th = 25000;
	}
	
	PCloud getframe(){
		hframe=true;
		ros::Rate r(2); // 10 hz
		
		while(hframe && ros::ok()){
			r.sleep();
			ros::spinOnce();
		}
		
		return cloudrec;
	}
	
	void sendcloudmsg(const PCloud& pcloud, bool haspoints=true){
		
		raposang_icp::pointcloud_t msg;
		sensor_msgs::PointCloud2 cloud_rostype;
		
		msg.header.frame_id = pcloud->header.frame_id ;
		msg.id = id++;
		msg.header.stamp    = ros::Time::now();
		
		pcl::toROSMsg(*pcloud,cloud_rostype); 
		msg.point_cloud = cloud_rostype;
		
		msg.T.translation.x = T(0,3);
		msg.T.translation.y = T(1,3);
		msg.T.translation.z = T(2,3);
		
		Eigen::Quaterniond q(T.block<3,3>(0,0));
		
		msg.T.rotation.x = q.x();
		msg.T.rotation.y = q.y();
		msg.T.rotation.z = q.z();
		msg.T.rotation.w = q.w();
		
		msg.hascloud = haspoints;
		
		pub_pointcloud.publish(msg);
	
	}
	
	void makemap_ICP();
	
	inline void setRansacThreshold(const double& th){rgbf.setRansacThreshold(th);}				//init
	inline void setDesiredP(const double& prob){rgbf.setDesiredP(prob);}						//init
	inline void setRThreshold(const float& THRESH_dr){icp.setRThreshold(THRESH_dr);}			//p2p
	inline void setTThreshold(const float& THRESH_dt){icp.setTThreshold(THRESH_dt);}			//p2p	
	inline void setDisThreshold(const float& D){icp.setDisThreshold(D);}						//p2p
	inline void setNiterations(const int& max_iterations){icp.setNiterations(max_iterations);}	//p2p
	inline void setMaxPairs(const int& MAXPAIRS){icp.setMaxPairs(MAXPAIRS);}					//p2p
	inline void setMinPairs(const int& MINPAIRS){icp.setMinPairs(MINPAIRS);}					//p2p
	//inline void setGicpEpsilon(double gicp_e){gicp_epsilon=gicp_e;}							//pl2pl
	//inline void setKcorrespondences(int k){k_correspondences=k;}								//pl2pl


private:	
	void get_cloud_Callback(const PCloud& pcloud){cloudrec=pcloud;hframe=false;}
	
	inline int test_cloud_size(const PCloud& pcloud){
		
		int npoints=0;
		
		for(unsigned int i=0; i<pcloud->size() ; i++){
			if (!isnan(pcloud->points[i].z)) npoints++;
		}
		
		return npoints;	
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
	
	T.setIdentity();
	
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take first frame
	cloud_s = getframe();
	sendcloudmsg(cloud_s);
	
	//take second frame	
	cloud_d = getframe();
			
	icp.setMcloud(cloud_s);		
	icp.setDcloud(cloud_d);
	rgbf.setScloud(cloud_s);		
	rgbf.setDcloud(cloud_d);
				
			
	float a1,a2;
	
	int ft, sw=0;
	for (int i=1; ros::ok() ; i++) {
		
		ROS_INFO("%d",test_cloud_size(cloud_d));			
		
		if (test_cloud_size(cloud_d) < psize_th){
			T.setIdentity();
			cloud_d = PCloud(new Cloud()); 
			sendcloudmsg(cloud_d, false);
			
			sw=1;
		}else{
		
			if(sw==1){
				rgbf.setNewinputCloud(cloud_d);
				icp.setNewICPinputCloud(cloud_d);
				sw=0;
			}else{			
					
				start = ros::Time::now();
				ft = rgbf.compute();
				end = ros::Time::now();
				a1 = (end-start).toSec();
				
				ROS_INFO("%d",ft);
				
				start = ros::Time::now();
				if ( ft > f_th ){	
					icp.align(rgbf.T,rgbf.getInliers()); 
				}else{
					icp.align(T); 
				}
				end = ros::Time::now();
				a2 = (end-start).toSec();
							
				//update transformation
				T = icp.returnTr();
			
				ROS_INFO("i= %d  |  RGB: %.6f seconds  |   ICP:  %.6f seconds   |  inliears= %d", i, a1, a2,ft);		
				
				sendcloudmsg(cloud_d);				
			}
		}
			
			cloud_d = getframe();
	
			//update icp clouds
			rgbf.setNewinputCloud(cloud_d);
			icp.setNewICPinputCloud(cloud_d);
		
	}
		
}



int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "demo_icp_keyframes");
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

	nh.param("sel",sel,0);
	
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
	
	demo d(pointcloudtopic);	
	d.setRansacThreshold(th);
	d.setDesiredP(prob);
	d.setRThreshold (THRESH_dr);
	d.setTThreshold (THRESH_dt);	
	d.setDisThreshold(D);
	d.setNiterations(max_iterations);
	d.setMaxPairs (MAXPAIRS);
	d.setMinPairs (MINPAIRS);
	//d.setGicpEpsilon(gicp_epsilon);
	//d.setKcorrespondences(k_correspondences);
		
		
	ROS_INFO("start!!");
	switch(sel){
		case 0: {d.makemap_ICP();} break;					//ICP point to point
	}
	ROS_INFO("done!!");
	
	return 0;
}
