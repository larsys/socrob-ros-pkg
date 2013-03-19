/*
 *  demo_publisher_odometry.cpp
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
#include <pcl/filters/voxel_grid.h>

#include "ros/ros.h"
#include "raposang_icp/pointcloud_t.h"
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "ICPInitializationBlock.h"
#include "ICPBlock.h"
#include "ICP_point2plane.h"
#include "ICP_plane2plane.h"

#include <Eigen/Dense>

 

typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


struct odometry{
	double time_stamp;
	Eigen::Vector3d t;
	Eigen::Quaterniond q;
	
	odometry(){
		time_stamp = 0.0;
		t << 0.0, 0.0, 0.0;
		 q = Eigen::Quaterniond(1,0,0,0);
	}
	
	Eigen::Matrix4d returnT(){
		Eigen::Matrix4d T;
		T.setIdentity();
		T.block<3,3>(0,0) = q.toRotationMatrix ();
		T.block<3,1>(0,3) = t;
		return T;	
	}
};


template <class ICPMode>
class demo{
protected:	
	RGBFeaturesInit rgbf;
	ICPMode icp; 
	bool useicp;
	
	//filter
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	bool filter;
	
	//threshold
	int f_th;
	int psize_th;
	
	bool hframe;
	int id;
	
	//for sync
	odometry *ring_buffer;
	int read_i, insert_i;
	int buffer_size;
	double time_c,time_o;
	
	//ROS
	ros::NodeHandle n;
	ros::Publisher  pub_pointcloud; //send frames
	ros::Subscriber sub_pointcloud,sub_odometry; //get frames
	
	
	PCloud cloudrec;
	odometry o_aux;
		
	Eigen::Matrix4d T;
	


public:
	demo(std::string _pointcloudtopic, std::string _odometrytopic){	
		T.setIdentity();
		pub_pointcloud = n.advertise< raposang_icp::pointcloud_t > ("/icp/world", 1);
		sub_pointcloud = n.subscribe(_pointcloudtopic.data(), 1, &demo::get_cloud_Callback, this);
		sub_odometry = n.subscribe(_odometrytopic.data(), 30, &demo::get_odometry_Callback, this);
		
		time_c=time_o= 0.0;
		id = read_i = insert_i = 0;
	}
	~demo(){
		free(ring_buffer);
	}
	
	PCloud getframe(){
		hframe=true;
		//ros::Rate r(3); // 10 hz
		
		ros::Time last_detection,now;
		
		last_detection = ros::Time::now();
		
		//std::cout << ros::Duration(0,500000000)  << "\n";
		
		while(ros::ok()){
			now = ros::Time::now();	
			if(now - last_detection < ros::Duration(0,500000000)){
				ros::spinOnce();
			}else{
				break;
			}
		}			

		hframe=true;
		while(hframe && ros::ok()){
			ros::spinOnce();
			//r.sleep();
		}
		
		syncOdoWithclouds();
		
		std::cout << "dif time = " << time_o - time_c << "\n";
		
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
	
	void updateTwithOdometry(odometry o_ant,odometry o_new){		
		//Eigen::Vector3d vec(0.1399949771,-0.02,0.9648094177);
		//Eigen::Quaterniond q(0.4148176325,-0.5691548531, 0.5831692902, -0.4048489396);
		Eigen::Vector3d vec(0.075, 0, 0.04);
		Eigen::Quaterniond q(0.9925,0,0.1219,0);
		
		Eigen::Matrix4d Tp, Tq;
		Tp.setIdentity();
		Tq.setIdentity();
		Tp.block<3,3>(0,0) << 0 , 0, 1, 
							 -1 , 0, 0,
							  0 , -1, 0;
		
		Tq.block<3,3>(0,0) = q.toRotationMatrix ();
		Tq.block<3,1>(0,3) = vec;
		
		T.setIdentity();
		//T.block<3,3>(0,0) = (o_new.q.inverse()*o_ant.q).toRotationMatrix ();
		//T.block<3,1>(0,3) = o_new.t-o_ant.t;
		
		T=o_ant.returnT().inverse()*o_new.returnT();
	
		T = T * Tq * Tp ;
		T = Tp.inverse()*Tq.inverse() * T;
	} 
	
	void makemap();
	
	
	void setFeaturesTh(int features_th){f_th = features_th;}
	void setCloudPointsMin(int cloud_points_min){psize_th = cloud_points_min;}
	void setBufferSize(int b_size){
		buffer_size = b_size;
		ring_buffer = (odometry*) malloc(buffer_size*sizeof(odometry));
		
		for (int i=0; i<buffer_size;i++){
			ring_buffer[i].time_stamp = 0;
			ring_buffer[i].t << 0.0,0.0,0.0;
			ring_buffer[i].q = Eigen::Quaterniond(1,0,0,0);
		}
	}
	void setRGBTh (double th){rgbf.setRansacThreshold(th);}
	void setRGBProb (double prob){rgbf.setDesiredP(prob);}
	void setFilterClouds(bool f, double s1=0.05,double s2=0.05,double s3=0.05){
		filter = f;
		grid.setLeafSize (s1, s2, s3);
	}	
	
	
protected:	
	void get_cloud_Callback(const PCloud& pcloud){
		cloudrec=pcloud;
		hframe=false;
		time_c = pcloud->header.stamp.toSec();
		//ROS_INFO("cloud: %.4f", time_c);
	}
	void get_odometry_Callback(const geometry_msgs::PoseStamped::ConstPtr& o){
		o_aux.t << o->pose.position.x, 
				   o->pose.position.y, 
				   o->pose.position.z;
		o_aux.q = Eigen::Quaterniond(o->pose.orientation.w,
								     o->pose.orientation.x,
								     o->pose.orientation.y,
								     o->pose.orientation.z);
		
		o_aux.time_stamp = o->header.stamp.toSec();					     

		ring_buffer[insert_i] = o_aux;
		//ROS_INFO("odo: %.4f %d", o_aux.time_stamp,insert_i);
		
		insert_i = (insert_i+1)%buffer_size;
		
		
		//std::cout << "estou a processar odo\n";
	}
	
	inline int test_cloud_size(const PCloud& pcloud){
		
		int npoints=0;
		
		for(unsigned int i=0; i<pcloud->size() ; i++){
			if (!isnan(pcloud->points[i].z)) npoints++;
		}
		
		return npoints;	
	}
	
	/*
	void syncOdoWithclouds(){
		
		double min_dif=100.0, dif;
		int best_ind=0;
		
		do{	
			dif=fabs(ring_buffer[read_i].time_stamp-time_c);
			if (dif <= min_dif){
				min_dif = dif;
				best_ind = read_i;
			}
			
			read_i = (read_i+1)%buffer_size;
			
		}while(read_i != insert_i);
		
		time_o = ring_buffer[best_ind].time_stamp;
		read_i = (best_ind+1)%buffer_size;
		o_aux = ring_buffer[best_ind];
	}
	*/	
	
	void syncOdoWithclouds(){
		
		double min_dif=100.0, dif;
		int best_ind=0;
		
		for(int i=0; i<buffer_size ; i++){
			
			dif=fabs(ring_buffer[i].time_stamp-time_c);
			if (dif < min_dif){
				min_dif = dif;
				best_ind = i;
			}
			
		}
		o_aux = ring_buffer[best_ind];
		time_o = ring_buffer[best_ind].time_stamp;
		
	}
	

};

class demo_icp: public demo<ICP>{
public:
	demo_icp(std::string _pointcloudtopic, std::string _odometrytopic,
										 bool _useicp,
										 const float& THRESH_dr,
										 const float& THRESH_dt,
										 const float& D,
										 const float& Dmax_c,
										 const int& max_iterations,
										 const int& MAXPAIRS,const int& MINPAIRS
										): demo(_pointcloudtopic,_odometrytopic){
		
		useicp=_useicp;
		icp.setRThreshold(THRESH_dr);
		icp.setTThreshold(THRESH_dt);
		icp.setDisThreshold(D);
		icp.setDismaxConst(Dmax_c);
		icp.setNiterations(max_iterations);
		icp.setMaxPairs(MAXPAIRS);
		icp.setMinPairs(MINPAIRS);
	}
};
class demo_icp_p2pl: public demo<ICP_p2plane>{
public:
	demo_icp_p2pl(std::string _pointcloudtopic, std::string  _odometrytopic,
										 bool _useicp,
										 const float& THRESH_dr,
										 const float& THRESH_dt,
										 const float& D,
										 const float& Dmax_c,
										 const int& max_iterations,
										 const int& MAXPAIRS,const int& MINPAIRS,
										 const int& k_correspondences					 
										): demo(_pointcloudtopic,_odometrytopic){
		
		useicp=_useicp;
		icp.setRThreshold(THRESH_dr);
		icp.setTThreshold(THRESH_dt);
		icp.setDisThreshold(D);
		icp.setDismaxConst(Dmax_c);
		icp.setNiterations(max_iterations);
		icp.setMaxPairs(MAXPAIRS);
		icp.setMinPairs(MINPAIRS);
		icp.setKcorrespondences(k_correspondences);
	}
};
class demo_icp_pl2pl: public demo<ICP_plane2plane>{
public:
	demo_icp_pl2pl(std::string _pointcloudtopic, std::string  _odometrytopic,
										 bool _useicp,
										 const float& THRESH_dr,
										 const float& THRESH_dt,
										 const float& D,
										 const float& Dmax_c,
										 const int& max_iterations,
										 const int& MAXPAIRS,const int& MINPAIRS,
										 const float& gicp_epsilon,
										 const int& k_correspondences					 
										): demo(_pointcloudtopic,_odometrytopic){
		
		useicp=_useicp;
		icp.setRThreshold(THRESH_dr);
		icp.setTThreshold(THRESH_dt);
		icp.setDisThreshold(D);
		icp.setDismaxConst(Dmax_c);
		icp.setNiterations(max_iterations);
		icp.setMaxPairs(MAXPAIRS);
		icp.setMinPairs(MINPAIRS);
		icp.setGicpEpsilon(gicp_epsilon);
		icp.setKcorrespondences(k_correspondences);
	}
};


template <class ICPMode>
void demo<ICPMode>::makemap(){
	
	ros::Time start, end;
	int oo=0, ff=0;
	
	T.setIdentity();

	PCloud cloud_s;
	PCloud cloud_d;
	PCloud cloud_filter;
	odometry o_ant;
	odometry o_new;
	bool icp_c=false;
		
	//take first frame
	cloud_s = getframe();
	o_ant=o_aux;
	
	if (filter){ 
		grid.setInputCloud (cloud_s);
		cloud_filter=PCloud(new Cloud());
		grid.filter (*cloud_filter);
	}
	
	sendcloudmsg(cloud_s);
	
	//take second frame	
	cloud_d = getframe();
	o_new=o_aux;
	
			
	rgbf.setScloud(cloud_s);		
	rgbf.setDcloud(cloud_d);
	
	if(filter){				
		icp.setMcloud(cloud_filter);	
		grid.setInputCloud (cloud_d);
		cloud_filter=PCloud(new Cloud());
		grid.filter (*cloud_filter);
		icp.setDcloud(cloud_filter);
	}else{
		icp.setMcloud(cloud_s);		
		icp.setDcloud(cloud_d);
	}		
			
	float a1,a2;
	
	int ft, sw=0;
	for (int i=1; ros::ok() ; i++) {
		
		ROS_INFO("cloud size = %d",test_cloud_size(cloud_d));			
		
		if (test_cloud_size(cloud_d) < psize_th){
			T.setIdentity();
			cloud_d = PCloud(new Cloud()); 
			sendcloudmsg(cloud_d, false);
			
			sw=1;
		}else{
		
			if(sw==1){
				rgbf.setNewinputCloud(cloud_d);
				if(filter){
					grid.setInputCloud (cloud_d);
					cloud_filter=PCloud(new Cloud());
					grid.filter (*cloud_filter);
					icp.setNewICPinputCloud(cloud_filter);
				}else{
					icp.setNewICPinputCloud(cloud_d);
				}
				sw=0;
			}else{			
					
				start = ros::Time::now();
				ft = rgbf.compute();
				end = ros::Time::now();
				a1 = (end-start).toSec();
				
				start = ros::Time::now();
				if ( ft > f_th ){	//ft
					T = rgbf.T;
					if(useicp)
						icp_c=icp.align(T);
						//icp_c=icp.align(T,rgbf.getInliers());
					ff++; 
				}else{
					updateTwithOdometry(o_ant,o_new);
					std::cout << T << "\n";
					if(useicp)
						icp_c=icp.align(T);
					oo++; 
				}
				end = ros::Time::now();
				a2 = (end-start).toSec();
							
				//update transformation
				if(useicp)
					if(icp_c) T = icp.returnTr();
			
				ROS_INFO("i= %d  |  RGB: %.6f seconds  |   ICP:  %.6f seconds   |  inliers= %d", i, a1, a2,ft);		
				
				sendcloudmsg(cloud_d);
			}
		}
			
			o_ant=o_new;
			cloud_d = getframe();
			o_new=o_aux;

			//update icp clouds
			rgbf.setNewinputCloud(cloud_d);
			if(filter){
				grid.setInputCloud (cloud_d);
				cloud_filter= PCloud(new Cloud());
				grid.filter (*cloud_filter);
				icp.setNewICPinputCloud(cloud_filter);
			}else{
				icp.setNewICPinputCloud(cloud_d);
			}
		
	}
	
	std::cout << "Odometria= " << oo << " "  << "Features= " << ff << "\n";
		
}


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "demo_icp_keyframes");
	ros::NodeHandle nh("~");	
	int sel;
	nh.param("sel",sel,0);

	//INI Thresholds
	double th,prob;
	nh.param("th",th,0.0001);
	nh.param("prob",prob,0.999);
	
	//ICP Thresholds
	double D, Dmax_c, THRESH_dr, THRESH_dt;
	int max_iterations, MAXPAIRS, MINPAIRS;
	nh.param("D",D,0.10);
	nh.param("Dmax_c",Dmax_c,20.0);
	nh.param("max_iterations",max_iterations,100);
	nh.param("THRESH_dr",THRESH_dr,0.5);
	nh.param("THRESH_dt",THRESH_dt,0.002); //0.004 fo pl2pl
	nh.param("MAXPAIRS",MAXPAIRS,4000);
	nh.param("MINPAIRS",MINPAIRS,2000);
	
	//ICP pl2pl Thresholds
	double gicp_epsilon;
	int k_correspondences;
	nh.param("gicp_epsilon",gicp_epsilon,0.0004);
	nh.param("k_correspondences",k_correspondences,20);

	//DEMO Thresholds
	int features_th, cloud_points_min, b_size;
	std::string pointcloudtopic, odometrytopic;
	double leaf_size;
	bool filter, useicp;
	nh.param("filter",filter,false);
	nh.param("leaf_size",leaf_size,0.05);
	nh.param("useicp",useicp,false);
	nh.param("features_th",features_th,4);
	nh.param("cloud_points_min",cloud_points_min,25000);
	nh.param("b_size",b_size,100);
	nh.param<std::string>("pointcloudtopic",pointcloudtopic,"/camera/rgb/points");
	nh.param<std::string>("odometrytopic",odometrytopic,"/odometry");
	
			
	ROS_INFO("start!!");
	switch(sel){
		//ICP point to point
		case 0: {
			demo_icp d(pointcloudtopic,odometrytopic, useicp,
					   THRESH_dr,THRESH_dt,D,Dmax_c,max_iterations,MAXPAIRS,MINPAIRS);
					   
			d.setFeaturesTh(features_th);
			d.setCloudPointsMin(cloud_points_min);
			d.setBufferSize(b_size);
			d.setFilterClouds(filter,leaf_size,leaf_size,leaf_size);
			d.setRGBTh(th);
			d.setRGBProb(prob);
					   
			d.makemap();
		} break;					
		//ICP point to plane
		case 1: {
			demo_icp_p2pl d(pointcloudtopic,odometrytopic, useicp,
					         THRESH_dr,THRESH_dt,D,Dmax_c,max_iterations,MAXPAIRS,MINPAIRS,
							 k_correspondences);
			
			d.setFeaturesTh(features_th);
			d.setCloudPointsMin(cloud_points_min);
			d.setBufferSize(b_size);
			d.setFilterClouds(filter);
			d.setRGBTh(th);
			d.setRGBProb(prob);					 
			
			d.makemap();
		} break;
		case 2: {
			demo_icp_pl2pl d(pointcloudtopic,odometrytopic, useicp,
					         THRESH_dr,THRESH_dt,D,Dmax_c,max_iterations,MAXPAIRS,MINPAIRS,
							 gicp_epsilon,k_correspondences);
			
			d.setFeaturesTh(features_th);
			d.setCloudPointsMin(cloud_points_min);
			d.setBufferSize(b_size);
			d.setFilterClouds(filter);
			d.setRGBTh(th);
			d.setRGBProb(prob);					 
			
			d.makemap();
		} break;
	} 
	ROS_INFO("done!!");
	
	return 0;
}
