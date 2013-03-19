
//test
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


#include "UtilitiesBlock.h"
#include "ros/ros.h"
#include <string>
#include <vector>
#include <Eigen/Dense>


//#include <Eigen/Dense>


typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;

ros::Time start, end;
std::string path_txt = "/home/pedrovieira/Desktop/datasets/";
std::string path_ply = "/home/pedrovieira/Desktop/results/";

std::string f1, fply1, fply2, fply3;


#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>







void main0(int argc, char* argv[]){
	
	f1 = path_txt + argv[2] + ".txt";
	
	fply1 = path_ply + argv[2] + ".ply";
	fply2 = path_ply + argv[2] + "_new.ply";
	
	
	PCloud cloud;
	PCloud cloud_f(new Cloud);
	LoadCloudFromTXT(f1.data(), cloud);
	
	start = ros::Time::now();
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_f);

	end = ros::Time::now();
	
	ROS_INFO("Noise Removal: %f", (end-start).toSec());
	
	PrintCloudToPLY(fply1.data(), cloud);
	PrintCloudToPLY(fply2.data(), cloud_f);
	
	std::cout <<"Number of points in cloud   " << cloud->size() << std::endl;
	std::cout <<"Number of points in cloud_f " << cloud_f->size() << std::endl;
	

}


void main1(int argc, char* argv[]){
	
	f1 = path_txt + argv[2] + ".txt";
	
	fply1 = path_ply + argv[2] + ".ply";
	fply2 = path_ply + argv[2] + "_new.ply";
	
	
	PCloud cloud;
	PCloud cloud_f(new Cloud);
	LoadCloudFromTXT(f1.data(), cloud);
	
	start = ros::Time::now();

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_f);

	end = ros::Time::now();
	
	ROS_INFO("Noise Removal: %f", (end-start).toSec());
	
	PrintCloudToPLY(fply1.data(), cloud);
	PrintCloudToPLY(fply2.data(), cloud_f);
	
	std::cout <<"Number of points in cloud   " << cloud->size() << std::endl;
	std::cout <<"Number of points in cloud_f " << cloud_f->size() << std::endl;
	

}


/**
 *  conclusoes: O voxel grid diminui bem o numero de pontos e o tempo é de 0.15 sec
 * 	o pass n\ao melhora o cloud significativamente
 * 
 * */

void main2(int argc, char* argv[]){
	
	double m_voxel_leah_size = 0.005;
	
	f1 = path_txt + argv[2] + ".txt";
	
	fply1 = path_ply + argv[2] + ".ply";
	fply2 = path_ply + argv[2] + "_new.ply";
	fply3 = path_ply + argv[2] + "_filter_new.ply";
	
	
	PCloud cloud;
	PCloud cloud_f(new Cloud);
	PCloud cloud_filter(new Cloud);
	LoadCloudFromTXT(f1.data(), cloud);
	
	start = ros::Time::now();
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	grid.setLeafSize(m_voxel_leah_size,m_voxel_leah_size,m_voxel_leah_size);
	grid.setInputCloud(cloud);
	grid.filter(*cloud_f);
	end = ros::Time::now();
	double t1=(end-start).toSec();
	ROS_INFO("Downsampling: %f",t1);
	
	
	start = ros::Time::now();
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	grid.setInputCloud(cloud_f);
	grid.filter(*cloud_filter);
	end = ros::Time::now();
	double t2=(end-start).toSec();
	ROS_INFO("filter: %f", t2);
	
	ROS_INFO("Down+filter: %f", t1+t2);
	
	PrintCloudToPLY(fply1.data(), cloud);
	PrintCloudToPLY(fply2.data(), cloud_f);
	PrintCloudToPLY(fply3.data(), cloud_filter);
	
	std::cout <<"Number of points in cloud   "      << cloud->size()        << std::endl;
	std::cout <<"Number of points in cloud_f "      << cloud_f->size()      << std::endl;
	std::cout <<"Number of points in cloud_filter " << cloud_filter->size() << std::endl;
	

}



void main3(int argc, char* argv[]){
	
	double m_voxel_leah_size = 0.005;
	
	f1 = path_txt + argv[2] + ".txt";
	
	fply1 = path_ply + argv[2] + ".ply";
	fply2 = path_ply + argv[2] + "_new.ply";
	fply3 = path_ply + argv[2] + "_with_normals.ply";
	
	
	PCloud cloud;
	PCloud cloud_f(new Cloud);
	
	LoadCloudFromTXT(f1.data(), cloud);
	
	start = ros::Time::now();
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	grid.setLeafSize(m_voxel_leah_size,m_voxel_leah_size,m_voxel_leah_size);
	grid.setInputCloud(cloud);
	grid.filter(*cloud_f);
	end = ros::Time::now();
	double t1=(end-start).toSec();
	ROS_INFO("Downsampling: %f",t1);
	
	start = ros::Time::now();
	pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree;
	kdtree.setInputCloud (cloud_f);	
	end = ros::Time::now();
	double t2=(end-start).toSec();
	ROS_INFO("Construct TREE: %f",t2);
	
	std::vector<Eigen::Vector3d> Normals;
	start = ros::Time::now();
	std::vector<int> kk_indices(20);
	std::vector<float> kk_distances(20);
	
	
	Eigen::Vector3d normal;
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
	int j;
	
	for(unsigned int i=0; i<cloud_f->size(); i++){
		Eigen::MatrixXd points(3, 20);
		mean << 0.0,0.0,0.0;
		
		kdtree.nearestKSearch  (i, 20, kk_indices, kk_distances); 
		
		for(j=0; j<kk_indices.size(); j++){
			points(0,j) = cloud_f->points[kk_indices[j]].x;
			points(1,j) = cloud_f->points[kk_indices[j]].y;
			points(2,j) = cloud_f->points[kk_indices[j]].z;
			
			mean = mean +  points.block<3,1>(0,j);
		}
		
		if(j!=20) points.conservativeResize(Eigen::NoChange, j);
		
		mean = mean/j;
		
		for (unsigned int k=0; k<j; k++)
			points.block<3,1>(0,k) = points.block<3,1>(0,k) - mean;
		
		cov = points * points.transpose();
		
		
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
	
		//the eigenvalues are sorted in increasing order
		normal = eigensolver.eigenvectors().col(0);
		
		if (normal(0)*cloud_f->points[i].x + normal(1)*cloud_f->points[i].y + normal(2)*cloud_f->points[i].z > 0) {
			normal[0] = -normal(0);
			normal[1] = -normal(1);
			normal[2] = -normal(2);
		}
		Normals.push_back(normal);
		
	}
	end = ros::Time::now();
	double t3=(end-start).toSec();
	ROS_INFO("Compute Normals: %f",t3);
	
	
	ROS_INFO("Down+TREE+Normals: %f", t1+t2+t3);
	
	PrintCloudToPLY(fply1.data(), cloud);
	PrintCloudToPLY(fply2.data(), cloud_f);
	PrintCloudToPLY(fply3.data(), cloud_f, Normals);
	
	std::cout <<"Number of points in cloud   "      << cloud->size()        << std::endl;
	std::cout <<"Number of points in cloud_f "      << cloud_f->size()      << std::endl;
	
}




int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "test_icp");
	ros::NodeHandle n;
	
	if (argc<2){
		std::cout<< "0 - remove by statistic | main(int) | file" << std::endl;
		std::cout<< "1 - remove by pass | main(int) | file" << std::endl;
		std::cout<< "2 - test voxel grid downsampling | main(int) | file" << std::endl;
		std::cout<< "3 - test normals with voxel grid downsampling | main(int) | file" << std::endl;

		return 0;
	}
	
	switch(atoi(argv[1])){
		case 0:{main0(argc, argv);}break;
		case 1:{main1(argc, argv);}break;
		case 2:{main2(argc, argv);}break;
		case 3:{main3(argc, argv);}break;
	}
	
	return 0;
}

