/*
 *  demo_icp.cpp
 *  
 *  Created by Pedro Vieira on 15/11/11.
 *  
 */

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"
#include <string>

#include "ICPBlock.h"
#include "ICP_plane2plane.h"
#include "ICPInitializationBlock.h"
#include "UtilitiesBlock.h"

std::string filename_path    = "/home/pedrovieira/Desktop/datasets/isr_dataset/";
std::string cameras_filename = "/home/pedrovieira/Desktop/results/CameraPoses.ply";

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class demo{
	ros::NodeHandle n;
	ros::Publisher  pub_pointcloud; //send frames
	
	PCloud cloudrec;
		
	Eigen::Matrix4d T;
	
	int acpf;
	int nframes;

	std::string filename;
	
public:
	demo(int nf,std::string f){
		T.setIdentity();
		pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("/icp/world", 1);
		acpf = 4;
		nframes=nf;
		filename = f;
	}
	
	void makemap_RGB();
	void makemap_RGB_ICP();
	void makemap_RGB_ICP_pl2pl2();
	void makemap_RGB_ICP_pl2pl2_vol();
	

private:	
	void get_cloud_Callback(int i){
		char f[500];
		sprintf(f,"%s_%d_.txt", filename.data(),i);
		
		LoadCloudFromTXT(f, cloudrec);
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

void demo::makemap_RGB(){
	
	ros::Rate r(2); // 10 hz
	ros::Time start, end;
	
	int i=0;
		
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	get_cloud_Callback(i);
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	
		
	ros::spinOnce();
	
	i++;
	get_cloud_Callback(i);	
	cloud_d = cloudrec;
		
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
		
	int k=1;
		
	float a1;
	int ft;
	while(i<nframes) {

		if(i>1){
			get_cloud_Callback(i);	
			cloud_d = cloudrec;
		}

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
				
				
			ApplyRTto(cloud_d);
			pub_pointcloud.publish(cloud_d);
						
		}else{
			k=1;
			ROS_INFO("Lost!! ");	
		}
			
		ros::spinOnce();	
		i++;	
	}
	
}

void demo::makemap_RGB_ICP(){
	
	ros::Time start, end;
	
	int i=0;
	
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_positions;
	Eigen::Vector3d p1(0.0,0.0,0.0);
	Eigen::Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
	
	PCloud cloud_s;
	PCloud cloud_d;
	
	//take 1 frame
	get_cloud_Callback(i);
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	ros::spinOnce();
	
	i++;
	get_cloud_Callback(i);	
	cloud_d = cloudrec;
		
	ICP icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
		
	int k=1;
		
	float a1,a2;
	int ft;
	while(i<nframes) {

		if(i>1){
			get_cloud_Callback(i);	
			cloud_d = cloudrec;
		}

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
				
			ApplyRTto(cloud_d);
			pub_pointcloud.publish(cloud_d);
			
			//update camera position
			p1 = T.block<3,3>(0,0)*camera_positions[0].first + T.block<3,1>(0,3);
			p2 = T.block<3,3>(0,0)*camera_positions[0].second + T.block<3,1>(0,3);
			camera_positions.push_back(std::make_pair(p1, p2));
							
		}else{
			k=1;
			ROS_INFO("Lost!! ");	
		}
		
		ros::spinOnce();
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(),camera_positions);
}

void demo::makemap_RGB_ICP_pl2pl2(){
	
	ros::Rate r(2); // 10 hz
	ros::Time start, end;
	
	int i=0;
	
	PCloud cloud_s;
	PCloud cloud_d;
	
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_positions;
	Eigen::Vector3d p1(0.0,0.0,0.0);
	Eigen::Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
	
	
	//take 1 frame
	get_cloud_Callback(i);
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";		

	ros::spinOnce();
	
	i++;
	get_cloud_Callback(i);	
	cloud_d = cloudrec;
		
	ICP_plane2plane icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
		
	int k=1;
		
	float a1,a2;
	int ft;
	while(i<nframes) {

		if(i>1){
			get_cloud_Callback(i);	
			cloud_d = cloudrec;
		}

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
		
		ros::spinOnce();
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(),camera_positions);
	
}

//com init velocidade
void demo::makemap_RGB_ICP_pl2pl2_vol(){
	
	ros::Rate r(2); // 10 hz
	ros::Time start, end;
	
	Eigen::Matrix4d T_backup;
	T_backup.setIdentity();
	int i=0;
	
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_positions;
	Eigen::Vector3d p1(0.0,0.0,0.0);
	Eigen::Vector3d p2(0.0,0.0,0.1);
	camera_positions.push_back(std::make_pair(p1, p2));
	
	PCloud cloud_s;
	PCloud cloud_d;
		
	//take 1 frame
	get_cloud_Callback(i);
	cloud_s = cloudrec;
	pub_pointcloud.publish(cloud_s);
	std::cout<< 0 << "\n";	

	ros::spinOnce();
	
	i++;
	get_cloud_Callback(i);	
	cloud_d = cloudrec;
	
	ICP_plane2plane icp(cloud_s,cloud_d);
	RGBFeaturesInit rgbf(cloud_s,cloud_d);
	
				
	float a1,a2;
	int ft;
	while(i<nframes) {
		
		if(i>1){
			get_cloud_Callback(i);	
			cloud_d = cloudrec;
		}

		ROS_INFO("%d ", i);	
		
		//update icp clouds
		rgbf.setNewinputCloud(cloud_d);
		icp.setNewICPinputCloud(cloud_d);
	
			
		start = ros::Time::now();
		ft = rgbf.compute();
		end = ros::Time::now();
		a1 = (end-start).toSec();
		
		start = ros::Time::now();	
		if (  ft > acpf ){	
			icp.align(rgbf.T,rgbf.getInliers()); 			
		}else{
			icp.align(T_backup); 
		}
		end = ros::Time::now();
		a2 = (end-start).toSec();
					
		//update transformation
		T_backup=icp.returnTr();
		T = T*icp.returnTr();
	
		//update camera position
		p1 = T.block<3,3>(0,0)*camera_positions[0].first + T.block<3,1>(0,3);
		p2 = T.block<3,3>(0,0)*camera_positions[0].second + T.block<3,1>(0,3);
		camera_positions.push_back(std::make_pair(p1, p2));
	
		ROS_INFO("RGB: %.6f seconds  |   ICP:  %.6f seconds   |  inliears= %d", a1, a2,ft);		
				
		ApplyRTto(cloud_d);
		pub_pointcloud.publish(cloud_d);
		
		ros::spinOnce();
		i++;
	}
	
	WriteCameraPLY(cameras_filename.data(),camera_positions);
	
}




//only RGB
void main0(int argc,char* argv[]){
	
	ROS_INFO("start!!");	
	
	std::string filename = argv[3];
	
	demo d(atoi(argv[2]), filename_path+filename);
	d.makemap_RGB();
	
	ROS_INFO("done!!");
	
}

//RGB + ICP
void main1(int argc,char* argv[]){
	
	ROS_INFO("start!!");
	
	std::string filename = argv[3];
	
	std::cout<< atoi(argv[2]) << "\n";
	
	demo d(atoi(argv[2]), filename_path+filename);
	d.makemap_RGB_ICP();
	
	ROS_INFO("done!!");
	
}

//RGB + ICP plane to plane
void main2(int argc,char* argv[]){
	
	ROS_INFO("start!!");
	
	std::string filename = argv[3];
	
	demo d(atoi(argv[2]), filename_path+filename);
	d.makemap_RGB_ICP_pl2pl2();
	
	ROS_INFO("done!!");
	
}

//RGB + ICP plane to plane
void main3(int argc,char* argv[]){
	
	ROS_INFO("start!!");
	
	std::string filename = argv[3];
	
	demo d(atoi(argv[2]), filename_path+filename);
	d.makemap_RGB_ICP_pl2pl2_vol();
	
	ROS_INFO("done!!");
	
}


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "demo_icp");
	ros::NodeHandle n;
	
	if(argc<2){
		std::cout << "todos eles --> nframes | filename" 			     << std::endl
				  << "0 - Make Map using RGB features" 			         << std::endl
				  << "1 - Make Map using RGB features and icp p2p"       << std::endl
				  << "2 - Make Map using RGB features and icp pl2pl"     << std::endl
				  << "3 - Make Map using RGB features and icp pl2pl vol" << std::endl;
		return 0;
	}
	
	switch(atoi(argv[1])){
		case 0: {main0(argc,argv);} break;
		case 1: {main1(argc,argv);} break;
		case 2: {main2(argc,argv);} break;
		case 3: {main3(argc,argv);} break;
	}
	
	return 0;
}
