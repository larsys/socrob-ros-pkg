/*
 *  worldmapBlock2.cpp
 *  
 *
 *  Created by Pedro Vieira on 02/14/12.
 * 
 * This file implements the world map topic
 */


#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <sys/stat.h>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "raposang_icp/pointcloud_t.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "ICPInitializationBlock.h"
#include "UtilitiesBlock.h"



typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class Printer{
	//ROS
	ros::Subscriber sub;
	ros::NodeHandle n;
	
	//filter
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	bool filter;
	
	//thresholds
	int key_th;
	
	//camera structures
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d> > camera_positions;
	
	//for key frames
	PCloud cloud_key;
	RGBFeaturesInit rgbf_key;
		
	Eigen::Matrix4d T_global;
	
	std::ofstream ply_file;
	long pos;
	unsigned int npoints;
	
	int i_folder;
	std::string folder_path;
	std::string folder_name;
	std::string filename;
	
	bool create_folder;
		
public:	
	Printer(std::string fpath){
		T_global.setIdentity();
		sub = n.subscribe("/icp/world", 30, &Printer::CloudPrinter_Callback_manyfiles,this); 
		cloud_key = PCloud(new Cloud());
		i_folder=1;
		
		create_folder = false;
		
		folder_path=fpath;
		mkdir ((folder_path+"/Map").data(),S_IRWXU | S_IRWXG | S_IROTH);
		
		createNewMapFolder();
		
		setkeyth();
		setRansacThreshold();
		setDesiredP();	
	}
	
	/*
	Printer(std::string fpath,std::string cloud_name){
		T_global.setIdentity();
		sub = n.subscribe("/icp/world", 30, &Printer::CloudPrinter_Callback_onefile,this);
		cloud_key = PCloud(new Cloud());
		i_folder=1;	
		npoints=0;
		
		
		
		ply_file.open (f.data());
		ply_file << "ply\n";
		ply_file << "format ascii 1.0\n";
		ply_file << "element vertex ";  pos=ply_file.tellp();
		ply_file << "                                           \n";
		
		ply_file << "property float x\n";
		ply_file << "property float y\n";
		ply_file << "property float z\n";
		ply_file << "property uchar red\n";
		ply_file << "property uchar green\n";
		ply_file << "property uchar blue\n";
		ply_file << "end_header\n";
	
	}
	*/
	
	void map(){
		ros::Rate r(3); // 10 hz
		r.sleep();
		ros::spin();
		
		WriteCameraPLY((folder_path+"/Map"+folder_name+"/cameras.ply").data(), camera_positions);
	}
	
	void closefile(){
		ply_file.seekp(pos);
		ply_file << npoints;
		ply_file.close();
	}
	
	inline void setpath(std::string f){filename = f;}	
	inline void setkeyth(const int& th=20){key_th=th;}
	inline void setRansacThreshold(const double& th=0.0001){rgbf_key.setRansacThreshold(th);}	//init
	inline void setDesiredP(const double& prob=0.999){rgbf_key.setDesiredP(prob);}				//init
	inline void setFilterClouds(bool f, double s1=0.05,double s2=0.05,double s3=0.05){
		filter = f;
		grid.setLeafSize (s1, s2, s3);
	}	
	
	
	
private:
	void CloudPrinter_Callback_onefile(const raposang_icp::pointcloud_t::ConstPtr& msg){
		
		Eigen::Matrix4d T; 
		T.setIdentity();
		bool iskeyframe;
		
		//get cloud
		PCloud pcloud; 
		pcl::fromROSMsg (msg->point_cloud, *pcloud);
		
		//get transformation
		T(0,3)=msg->T.translation.x;
		T(1,3)=msg->T.translation.y;
		T(2,3)=msg->T.translation.z;		
		Eigen::Quaterniond q(msg->T.rotation.w,msg->T.rotation.x,msg->T.rotation.y,msg->T.rotation.z);
		T.block<3,3>(0,0) = q.toRotationMatrix(); 
		
		//check if keyframe
		iskeyframe=checkIfIsNewKeyframe(pcloud);
		
		//update Global T
		T_global = T_global*T;
		
		if(iskeyframe){
			//add new camera
			addcamera();
			for(unsigned int k=0;k<pcloud->size();k++) {
				if (!isnan(pcloud->points[k].z) && ((int) pcloud->points[k].r != 0 && (int) pcloud->points[k].g !=0 && (int) pcloud->points[k].b != 0 )){
					Eigen::Vector3d p_aux;
					
					p_aux(0) = pcloud->points[k].x*T_global(0,0) + pcloud->points[k].y*T_global(0,1) + pcloud->points[k].z*T_global(0,2) + T_global(0,3); 
					p_aux(1) = pcloud->points[k].x*T_global(1,0) + pcloud->points[k].y*T_global(1,1) + pcloud->points[k].z*T_global(1,2) + T_global(1,3); 
					p_aux(2) = pcloud->points[k].x*T_global(2,0) + pcloud->points[k].y*T_global(2,1) + pcloud->points[k].z*T_global(2,2) + T_global(2,3); 
					
					ply_file << p_aux(0) << " " << p_aux(1) << " " << p_aux(2) << " "
							 <<(int) pcloud->points[k].r << " " << (int)pcloud->points[k].g << " " << (int) pcloud->points[k].b << "\n";
					
					npoints++;
				}
			}
		}
	}
	void CloudPrinter_Callback_manyfiles(const raposang_icp::pointcloud_t::ConstPtr& msg){
		
		if(msg->hascloud){
		
			Eigen::Matrix4d T; 
			T.setIdentity();
			bool iskeyframe;
			
			//get cloud
			PCloud pcloud= PCloud(new Cloud()); 
			pcl::fromROSMsg (msg->point_cloud, *pcloud);
			
			//get transformation
			T(0,3)=msg->T.translation.x;
			T(1,3)=msg->T.translation.y;
			T(2,3)=msg->T.translation.z;		
			Eigen::Quaterniond q(msg->T.rotation.w,msg->T.rotation.x,msg->T.rotation.y,msg->T.rotation.z);
			T.block<3,3>(0,0) = q.toRotationMatrix(); 
			
			//check if keyframe
			iskeyframe=checkIfIsNewKeyframe(pcloud);
			
			//update Global T
			T_global = T_global*T;
			
			if(iskeyframe){
				char dummy[12];
				sprintf (dummy, "%d", msg->id);
				filename = folder_path + "/Map" + folder_name + "/cloud_" + dummy + ".ply";
				
				//add new camera
				addcamera();
				
				ApplyRTto(pcloud);
				PrintCloudToPLY(filename.data(), pcloud);
				create_folder = true;
			}
		}else{
			if(create_folder){
				WriteCameraPLY((folder_path+"/Map"+folder_name+"/cameras.ply").data(), camera_positions);
				camera_positions.clear();
				cloud_key = PCloud(new Cloud());
				createNewMapFolder();
				create_folder = false;
			}
		}
	}
	
	inline bool checkIfIsNewKeyframe(const PCloud& cloud_new);
	inline void addcamera();
	inline void ApplyRTto(PCloud &pc);
	inline bool createNewMapFolder();
	
};



bool Printer::checkIfIsNewKeyframe(const PCloud& cloud_new){	
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

void Printer::addcamera(){
		Eigen::Vector3d p1(0.0,0.0,0.0);
		Eigen::Vector3d p2(0.0,0.0,0.1);
		
		p1 = T_global.block<3,3>(0,0)*p1 + T_global.block<3,1>(0,3);
		p2 = T_global.block<3,3>(0,0)*p2 + T_global.block<3,1>(0,3);
		camera_positions.push_back(std::make_pair(p1, p2));	
		
	}
	
	
void Printer::ApplyRTto(PCloud &pc){
		
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	PCloud pcloud = PCloud(new Cloud());
	PCloud pcloud_filter = PCloud(new Cloud());
	
	if(filter){
		grid.setInputCloud (pc);
		grid.filter (*pcloud_filter);
		pc = pcloud_filter;
	}

	pcloud->header.stamp = pc->header.stamp;
	pcloud->header.frame_id = pc->header.frame_id;
	pcloud->is_dense = pc->is_dense;
	
	pcloud->width = pc->width;
	pcloud->height = pc->height;	
	
	pcloud->points.resize(pc->size());		
		
	for(unsigned int i=0;i<pc->size();i++) {
		if (!isnan(pc->points[i].z)){
			pcloud->points[i].x = pc->points[i].x*T_global(0,0) + pc->points[i].y*T_global(0,1) + pc->points[i].z*T_global(0,2) + T_global(0,3);
			pcloud->points[i].y = pc->points[i].x*T_global(1,0) + pc->points[i].y*T_global(1,1) + pc->points[i].z*T_global(1,2) + T_global(1,3);
			pcloud->points[i].z = pc->points[i].x*T_global(2,0) + pc->points[i].y*T_global(2,1) + pc->points[i].z*T_global(2,2) + T_global(2,3);	
		}else{
			pcloud->points[i].x = bad_point;
			pcloud->points[i].y = bad_point;
			pcloud->points[i].z = bad_point;	
		}
		pcloud->points[i].r = pc->points[i].r;
		pcloud->points[i].g = pc->points[i].g;
		pcloud->points[i].b = pc->points[i].b;
	}
	
	pc = pcloud;
}

bool Printer::createNewMapFolder(){
	char dummy[12];
	std::string number;
	
	sprintf(dummy,"%d",i_folder); 
	number = dummy;
	
	folder_name = "/map" + number;
	i_folder++;	
	

	return mkdir((folder_path+"/Map"+folder_name).data(),S_IRWXU | S_IRWXG | S_IROTH);
}


int main(int argc, char *argv[]){
	
	ros::init(argc, argv, "world_node");
	ros::NodeHandle nh("~");
	
	
	int sel;
	int key_th;
	double th;
	double prob;
	double leaf_size;
	bool filter;
	
	std::string ply_filename;
	std::string ply_filepath;
	
	nh.param("sel",sel,0);
	nh.param("filter",filter,true);
	nh.param("leaf_size",leaf_size,0.05);
	
	nh.param("key_th",key_th,20);
	nh.param("th",th,0.0001);
	nh.param("prob",prob,0.999);
	
	nh.param<std::string>("ply_filepath",ply_filepath,"/home/");
	nh.param<std::string>("ply_filename",ply_filename,"/home/cloud.ply");
	
		
	switch(sel){
		case 0:{						//many files
			Printer p(ply_filepath);
			p.setkeyth(key_th);
			p.setRansacThreshold(th);
			p.setDesiredP(prob);
			p.setFilterClouds(filter,leaf_size,leaf_size,leaf_size);	
			p.map();	
		}break;
		/*case 1:{						//1 file
			Printer p(ply_filename);
			p.setkeyth(key_th);
			p.setRansacThreshold(th);
			p.setDesiredP(prob);	
			p.map();
			p.closefile();
		}break;*/
	}

}
	





