

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "UtilitiesBlock.h"
#include "ros/ros.h"



ros::Publisher pub_pointcloud;
char filename_txt[100];
char filename_ply[100];


void send_to_world_Callback(const PCloud& pcloud){

	pub_pointcloud.publish(pcloud);

}


void write_txt_cloud_Callback(const PCloud& pcloud){

	SaveCloudToTXT(filename_txt,pcloud);

}



//tira 1 frame do kinect e envia-o para o topico /world 
void main0(int argc, char* argv[]){
	
	ros::init(argc, argv, "test_world_node");
	ros::NodeHandle n;
	
	pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("world", 1);
	
	ros::Subscriber sub = n.subscribe("/pointcloud", 1, send_to_world_Callback);
	
	ros::Rate r(1); // 10 hz
	r.sleep();
	
	ros::spinOnce();
	//ros::spin();

}

//tira 1 frame do kinect escreve o num ficheiro txt
void main1(int argc, char* argv[]){
	
	if(argc<3){
		std::cout<<"txt filename is missing\n";
		return;
	}
	
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
	
	ros::init(argc, argv, "test_world_node");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/pointcloud", 1, write_txt_cloud_Callback);
	
	ros::Rate r(1); // 10 hz
	r.sleep();
	
	ros::spinOnce();
	//ros::spin();

}

//Load a .txt file and send a cloud to /world topic \n 
void main2(int argc, char* argv[]){
	
	if(argc<3){
		std::cout<<"txt filename is missing\n";
		return;
	}
	
	ros::init(argc, argv, "test_world_node");
	ros::NodeHandle n;
	
	pub_pointcloud = n.advertise< sensor_msgs::PointCloud2 > ("world", 1);
	
	//my part
	PCloud pcloud;
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
	LoadCloudFromTXT(filename_txt,pcloud);
	//end my part
	
	send_to_world_Callback(pcloud);
	
	
	ros::spinOnce();
	//ros::spin();

}

//Load a .txt file and write a ply file \n 
void main3(int argc, char* argv[]){
	
	if(argc!=4){
		std::cout<<"txt or ply filename is missing\n";
		return;
	}
	
	ros::init(argc, argv, "test_world_node");
	ros::NodeHandle n;
	
	
	//my part
	PCloud pcloud;
	sprintf (filename_txt, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
	LoadCloudFromTXT(filename_txt,pcloud);
	
	sprintf (filename_ply, "/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/%s", argv[3]);
	PrintCloudToPLY(filename_ply, pcloud);
	
}



int main(int argc, char* argv[]){
	
	if(argc<2){
		std::cout << "0 - tira 1 frame do kinect e envia-o para o topico /world\n"
				  << "1 - tira 1 frame do kinect escreve o num ficheiro txt | cloud.txt\n"
				  << "2 - Load a .txt file and send a cloud to /world topic | cloud.txt \n"
				  << "3 - Load a .txt file and write a ply file | cloud.txt cloud.ply \n\n";
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

