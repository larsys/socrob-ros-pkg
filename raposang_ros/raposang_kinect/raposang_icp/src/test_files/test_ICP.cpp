
//test
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "UtilitiesBlock.h"
#include "ros/ros.h"
#include <string>

#include "pcl/kdtree/organized_data.h"

#include <Eigen/Dense>

#include "ICPInitializationBlock.h"
#include "ICPBlock.h"
#include "ICP_plane2plane.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;

ros::Time start, end;
//std::string path_txt = "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/";
//std::string path_ply = "/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/";
std::string path_txt = "/home/pedrovieira/Desktop/datasets/ ";
std::string path_ply = "/home/pedrovieira/Desktop/results/";
std::string f1,f2, fply1, fply2, fply3;
//datasets
std::string file_s_pose = "/home/pedrovieira/Desktop/results/Arpose/cloud_s_truth_pose.txt";
std::string file_d_pose = "/home/pedrovieira/Desktop/results/Arpose/cloud_d_truth_pose.txt";


#include "test_files/test_ICP_p2p.hpp"
//#include "test_files/test_ICP_pl2pl.hpp"
#include "test_files/test_ICP_INIT.hpp"
#include "DatasetAnalyzer.h"


//ICP normal
void main0(int argc, char* argv[]){
	
	f1 = path_txt + argv[2] + ".txt";
	f2 = path_txt + argv[3] + ".txt";
	
	fply1 = path_ply + argv[2] + ".ply";
	fply2 = path_ply + argv[3] + ".ply";
	fply3 = path_ply + argv[3] + "_new.ply";
	
	//TestICP test(f1.data() ,f2.data());
	//start = ros::Time::now();
	//test.align();
	//test.test_tree();
	//test.test_cloud_mize();	
	//end = ros::Time::now();
	
	
	PCloud cloud_m;
	PCloud cloud_d;
	LoadCloudFromTXT(f1.data(), cloud_m);
	LoadCloudFromTXT(f2.data(), cloud_d);	
	start = ros::Time::now();
	RGBFeaturesInit rgbf(cloud_m,cloud_d);
	ICP icp(cloud_m,cloud_d);
	rgbf.compute();
	icp.align(rgbf.T,rgbf.getInliers());
	end = ros::Time::now();
	
	ROS_INFO("ICP time: %f", (end-start).toSec());
	
	PrintCloudToPLY(fply1.data(), cloud_m);
	PrintCloudToPLY(fply2.data(), cloud_d);
	PrintCloudToPLY(fply3.data(), cloud_d,icp.returnTr());
	
	Dataset_analyzer dataset(file_s_pose,file_d_pose);
	Eigen::Matrix4d T_icp= icp.returnTr();
	Eigen::Vector3d vec_truth = dataset.returnTran();
	
	Eigen::Vector3d icp_angles;
	Eigen::Vector3d truth_angles;
	
	icp_angles(0) = atan2(T_icp(2,1), T_icp(2,2));
	icp_angles(1) = asin(-T_icp(2,0));
	icp_angles(2) = atan2(T_icp(1,0), T_icp(0,0));
	dataset.returnAngles_rad(truth_angles(0),truth_angles(1),truth_angles(2));

	std::cout << "Erro Rotação: " << fabs(icp_angles.norm() - truth_angles.norm())*180/3.1415926535 << " degrees" << std::endl;
	std::cout << "Erro translaçao: " << fabs(vec_truth.norm() - T_icp.block<3,1>(0,3).norm()) << " m" << std::endl;
		
	//std::cout << T_icp << "\n";
	//Eigen::Matrix4d T= rgbf.T;
}

//ICP plane 2 plane
void main1(int argc, char* argv[]){
		
	f1 = path_txt + argv[2] + ".txt";
	f2 = path_txt + argv[3] + ".txt";
	
	fply1 = path_ply + argv[2] + ".ply";
	fply2 = path_ply + argv[3] + ".ply";
	fply3 = path_ply + argv[3] + "_new.ply";
	
	//std::vector<std::pair<int,int> > dumy;							/////////tira isto
			
	PCloud cloud_m;
	PCloud cloud_d;
	LoadCloudFromTXT(f1.data(), cloud_m);
	LoadCloudFromTXT(f2.data(), cloud_d);	
	start = ros::Time::now();
	RGBFeaturesInit rgbf(cloud_m,cloud_d);
	ICP_plane2plane icp_pl2pl(cloud_m,cloud_d);
	rgbf.compute();
	icp_pl2pl.align(rgbf.T,rgbf.getInliers());
	//icp_pl2pl.align(rgbf.T,dumy);									/////////tira isto
	end = ros::Time::now();
	
	ROS_INFO("ICP time: %f", (end-start).toSec());
	ROS_INFO("RGB features: %d", rgbf.getInliers().size());
	
	PrintCloudToPLY(fply1.data(), cloud_m);
	PrintCloudToPLY(fply2.data(), cloud_d);
	PrintCloudToPLY(fply3.data(), cloud_d,icp_pl2pl.returnTr());
		
	Dataset_analyzer dataset(file_s_pose,file_d_pose);
	Eigen::Matrix4d T_icp= icp_pl2pl.returnTr();
	Eigen::Vector3d vec_truth = dataset.returnTran();
	
	Eigen::Vector3d icp_angles;
	Eigen::Vector3d truth_angles;
	
	icp_angles(0) = atan2(T_icp(2,1), T_icp(2,2));
	icp_angles(1) = asin(-T_icp(2,0));
	icp_angles(2) = atan2(T_icp(1,0), T_icp(0,0));
	dataset.returnAngles_rad(truth_angles(0),truth_angles(1),truth_angles(2));

	std::cout << "Erro Rotação: " << fabs(icp_angles.norm() - truth_angles.norm())*180/3.1415926535 << " degrees" << std::endl;
	std::cout << "Erro translaçao: " << fabs(vec_truth.norm() - T_icp.block<3,1>(0,3).norm()) << " m" << std::endl;
		
	//std::cout << T_icp << "\n";
	//Eigen::Matrix4d T= rgbf.T;	
}

//RGB features
void main2(int argc, char* argv[]){
	f1 = path_txt + argv[3] + ".txt";
	f2 = path_txt + argv[4] + ".txt";
	
	fply1 = path_ply + argv[3] + ".ply";
	fply2 = path_ply + argv[4] + ".ply";
	fply3 = path_ply + argv[4] + "_new.ply";
	
	TestINI t(f1.data() ,f2.data());
	
	//double p = (double)atoi(argv[2])/100.0;
	//double tr = (double)atoi(argv[3])/(double)atoi(argv[4]);
	
	//t.setDesiredP(p);
	//t.setRansacThreshold(tr); 
	//t.setMinMatchDist(3);
	
	int f=0;
	switch(atoi(argv[2])){
		case 0:{f=t.test_compute();}break;
		case 1:{f=t.test_algTime();}break;
		case 2:{f=t.test_distances();}break;
	}
	
	if(f >= 4) t.Writeclouds(fply1.data(),fply2.data());
	
}


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "test_icp");
	ros::NodeHandle n;
	
	if (argc<2){
		std::cout<< "0 - Testa icp | main(int) | s_file | d_file " << std::endl;
		std::cout<< "1 - Testa icp plane 2 plane | main(int) | s_file | d_file" << std::endl;
		std::cout<< "2 - Testa RGB distances | main(int) | choose(int) | s_file | d_file" << std::endl;
		return 0;
	}
	
	switch(atoi(argv[1])){
		case 0:{main0(argc, argv);}break;
		case 1:{main1(argc, argv);}break;
		case 2:{main2(argc, argv);}break;
	}
	
	return 0;
}

