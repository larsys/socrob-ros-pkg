/*
 *  worldmapBlock.cpp
 *  
 *
 *  Created by Pedro Vieira on 10/24/11.
 * 
 * This file implements the world map topic
 */


#include <iostream>
#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <fstream>
#include <string>
#include <vector>

#include "UtilitiesBlock.h"



typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCloud;


class Printer{
	ros::Subscriber sub;
	ros::NodeHandle n;
	
	std::ofstream ply_file;
	long pos;
	unsigned int i;
	
	std::string filename;
	
		
public:	
	Printer(){
		i=0;
		sub = n.subscribe("/icp/world", 20, &Printer::CloudPrinter_Callback_manyfiles,this);
	}
	Printer(std::string f){
		i=0;
		sub = n.subscribe("/icp/world", 20, &Printer::CloudPrinter_Callback_onefile,this);
		
		
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
	
	void map(){
		ros::Rate r(3); // 10 hz
		r.sleep();
		ros::spin();
	}
	
	void closefile(){
		ply_file.seekp(pos);
		ply_file << i;
		ply_file.close();
	}
	
	void Setpath(std::string f){
		filename = f;
	}
	
private:
	void CloudPrinter_Callback_onefile(const PCloud& pcloud){
		for(unsigned int k=0;k<pcloud->size();k++) {
			if (!isnan(pcloud->points[k].z) && ((int) pcloud->points[k].r != 0 && (int) pcloud->points[k].g !=0 && (int) pcloud->points[k].b != 0 )){
				ply_file << pcloud->points[k].x << " " << pcloud->points[k].y << " " << pcloud->points[k].z << " "
						 <<(int) pcloud->points[k].r << " " << (int)pcloud->points[k].g << " " << (int) pcloud->points[k].b << "\n";
				i++;
			}
		}
	}
	void CloudPrinter_Callback_manyfiles(const PCloud& pcloud){
		
		char fname[120]; 
		sprintf (fname, "%scloud_%u.ply",filename.data(), i);
	
		i++;
		
		PrintCloudToPLY(fname, pcloud);
	}
	
};



//many files
void main0(std::string filepath){
	Printer p;
	p.Setpath(filepath);
	p.map();
}

//1 file
void main1(std::string filename){	
	
	Printer p(filename);
	p.map();
	p.closefile();
}

int main(int argc, char *argv[]){
	
	int sel;
	std::string ply_filename;
	std::string ply_filepath;
	
	ros::init(argc, argv, "world_node");
	ros::NodeHandle nh("~");
	nh.param("sel",sel,0);
	
	nh.param<std::string>("ply_filepath",ply_filepath,"/home/");
	nh.param<std::string>("ply_filename",ply_filename,"/home/cloud.ply");
	
		
	switch(sel){
		case 0:{main0(ply_filepath);}break;
		case 1:{main1(ply_filename);}break;
	}

}
	





