
#include <iostream>
#include <fstream>
#include <vector>

#include <cv.h> 
#include <opencv2/highgui/highgui.hpp> 

#include "ros/ros.h"

#include "UtilitiesBlock.h"
#include "ICPInitializationBlock.h"
#include <pcl_ros/point_cloud.h>


class Test_RGBFeaturesInit : public RGBFeaturesInit{	
	
	PCloud cloudrec;
	
public:
	Test_RGBFeaturesInit() : RGBFeaturesInit(){}
	Test_RGBFeaturesInit(char* filecl_s, char* filecl_d){
		LoadCloudFromTXT(filecl_s,cloud_s);
		LoadCloudFromTXT(filecl_d,cloud_d);	
	}

	int compute_ORB(); 
	int compute_surf(bool debug=false); 
	
	void makemap(int nframes);
	
	void PrintCloudsToPLY(char* filename);
	
	void streamVideo();
	
private:
	inline void computeSURFfeatures(cv::Mat im , cv::Mat &desc , std::vector<cv::KeyPoint> &kp);

	inline void imshow(char* window_name, cv::Mat im, bool block=false);
	inline void featureshow(char* window_name, cv::Mat im, std::vector<cv::KeyPoint> kp, bool block=false);

	void get_cloud_Callback(const PCloud& pcloud);
};

int Test_RGBFeaturesInit::compute_ORB(){
	
	ros::Time start;
	ros::Time end;
	
	start = ros::Time::now();	
	cv::Mat im_s = extractRGBfromCloud(cloud_s);
	cv::Mat im_d = extractRGBfromCloud(cloud_d);
	end = ros::Time::now();
	printf("Extract RGB from clouds: %.6f seconds\n", (end-start).toSec());
	
	imshow("image_s",im_s);
	imshow("image_d",im_d,true);
	
	cv::Mat desc_s;  
	cv::Mat desc_d;  
	std::vector<cv::KeyPoint> kp_s;
	std::vector<cv::KeyPoint> kp_d;
	
	start = ros::Time::now();
	computeORBfeatures(im_s , desc_s , kp_s);
	computeORBfeatures(im_d , desc_d , kp_d);
	end = ros::Time::now();
	printf("Compute ORB features 2 clouds: %.6f seconds\n", (end-start).toSec());
	
	featureshow("image_s", im_s, kp_s);
	featureshow("image_d", im_d, kp_d,true);
	
	// matches[i].queryIdx --> kp_s | matches[i].trainIdx --> kp_d
	start = ros::Time::now();
	cv::BruteForceMatcher<cv::L2<uchar> > matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(desc_s,desc_d,matches);
	end = ros::Time::now();
	printf("ORB Matching: %.6f seconds\n", (end-start).toSec());
	
	cv::Mat imageMatches;
	cv::drawMatches(im_s,kp_s,im_d,kp_d,matches,imageMatches,cv::Scalar(255,0,0));
	imshow("matches",imageMatches,true);
	
	start = ros::Time::now();
	make3Dpairs(kp_s, kp_d, matches);
	end = ros::Time::now();
	printf("Make 3D pairs: %.6f seconds\n", (end-start).toSec());
	
	start = ros::Time::now();
	TakeOutOutliers();	
	end = ros::Time::now();
	printf("RANSAC: %.6f seconds\n", (end-start).toSec());
	

	std::cout<<"Matches: "<< matches.size() << "\n";
	std::cout<<"Matches 3d: "<< matches3D.size() << "\n";
	
	return inliers.size();
	
} 

int Test_RGBFeaturesInit::compute_surf(bool debug){
	
	ros::Time start;
	ros::Time end;
	
	if(debug) start = ros::Time::now();
	
	cv::Mat im_s = extractRGBfromCloud(cloud_s);
	cv::Mat im_d = extractRGBfromCloud(cloud_d);
	
	if(debug){
		end = ros::Time::now();
		printf("Extract RGB from clouds: %.6f seconds\n", (end-start).toSec());
	}
	
	
	if(debug){
		imshow("image_s",im_s);
		imshow("image_d",im_d,true);
	}
	
	
	cv::Mat desc_s;  
	cv::Mat desc_d;  
	std::vector<cv::KeyPoint> kp_s;
	std::vector<cv::KeyPoint> kp_d;
	
	if(debug) start = ros::Time::now();
	
	computeSURFfeatures(im_s , desc_s , kp_s);
	computeSURFfeatures(im_d , desc_d , kp_d);
	
	if(debug){
		end = ros::Time::now();
		printf("Compute SURF features 2 clouds: %.6f seconds\n", (end-start).toSec());
	}
	
	if(debug){
		featureshow("image_s", im_s, kp_s);
		featureshow("image_d", im_d, kp_d,true);
	}
	
	
	// matches[i].queryIdx --> kp_s | matches[i].trainIdx --> kp_d
	//cv::BruteForceMatcher<cv::L2<float> > matcher;
	
	if(debug) start = ros::Time::now();
	
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(desc_s,desc_d,matches);
	
	if(debug){
		end = ros::Time::now();
		printf("SURF Matching: %.6f seconds\n", (end-start).toSec());
	}
	

	if(debug){
		cv::Mat imageMatches;
		cv::drawMatches(im_s,kp_s,im_d,kp_d,matches,imageMatches,cv::Scalar(255,0,0));
		imshow("matches",imageMatches,true);
	}
	
	if(debug) start = ros::Time::now();
	
	make3Dpairs(kp_s, kp_d, matches);
	
	if(debug){
		end = ros::Time::now();
		printf("Make 3D pairs: %.6f seconds\n", (end-start).toSec());
	}
	
	if(debug) start = ros::Time::now();
	
	TakeOutOutliers();	
	
	if(debug){
		end = ros::Time::now();
		printf("RANSAC: %.6f seconds\n", (end-start).toSec());
		std::cout<<"Features: "<< matches3D.size() << "\n";	
	}
	
	
	return inliers.size();
	
} 

void Test_RGBFeaturesInit::makemap(int nframes){
	
	ros::NodeHandle n;
	ros::Rate r(2); // 10 hz
	ros::Subscriber sub_pointcloud = n.subscribe("/pointcloud", 1, &Test_RGBFeaturesInit::get_cloud_Callback, this);
		
	long pos;
	unsigned int npoints=0;
	Eigen::Vector3d p;
	
	Eigen::Matrix3d R_c = Eigen::MatrixXd::Identity(3,3);
	Eigen::Vector3d T_c(0.0,0.0,0.0);
	
	//take frames
	r.sleep();
	ros::spinOnce();
	cloud_s = cloudrec;

	
	std::ofstream ply_file("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/world.ply");
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	ply_file << "element vertex ";  pos=ply_file.tellp();
	ply_file << "                   \n";
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(int v=0;v<cloud_s->height;v++) {	  //480
		for(int u=0;u<cloud_s->width;u++) {   //640
			if (!isnan((*cloud_s)(u,v).z)){
				ply_file << (*cloud_s)(u,v).x << " " << (*cloud_s)(u,v).y << " " << (*cloud_s)(u,v).z << " "
						 <<(int) (*cloud_s)(u,v).r << " " << (int) (*cloud_s)(u,v).g << " " << (int) (*cloud_s)(u,v).b << "\n";
				npoints++;
			}
		}
	}
	
	int in;
	for(int i=1; i<nframes; i++){

		if(i!=1) cloud_s=cloud_d;
		
		std::cout<< i;
	
		r.sleep();	
		ros::spinOnce();
		cloud_d = cloudrec;
			
		//in=compute_ORB();
		in=compute();
		std::cout<<" inliers= " << in << "\n\n";
		if (in>4){
			
			//update transformation
			T_c = T_c + R_c*T.topRightCorner<3,1>(); //T = R*T_new + T;
			R_c = R_c*T.topLeftCorner<3,3>();	     //R = R*R_new;
		
		
			for(int v=0;v<cloud_d->height;v++) {	  //480
				for(int u=0;u<cloud_d->width;u++) {   //640
					if (!isnan((*cloud_d)(u,v).z)){
				
						p(0) = (*cloud_d)(u,v).x*R_c(0,0) + (*cloud_d)(u,v).y*R_c(0,1) + (*cloud_d)(u,v).z*R_c(0,2) + T_c(0);
						p(1) = (*cloud_d)(u,v).x*R_c(1,0) + (*cloud_d)(u,v).y*R_c(1,1) + (*cloud_d)(u,v).z*R_c(1,2) + T_c(1);
						p(2) = (*cloud_d)(u,v).x*R_c(2,0) + (*cloud_d)(u,v).y*R_c(2,1) + (*cloud_d)(u,v).z*R_c(2,2) + T_c(2);
								
						ply_file << p(0) << " " << p(1) << " " << p(2) << " "
								 <<(int) (*cloud_d)(u,v).r << " " << (int) (*cloud_d)(u,v).g << " " << (int) (*cloud_d)(u,v).b << "\n";
						npoints++;
					}
				}
			}
		}
		matches3D.clear();
		inliers.clear();
		T = Eigen::MatrixXd::Identity(4,4);	
	}	
	
	ply_file.seekp(pos);
	ply_file << npoints;
	ply_file.close();
}

void Test_RGBFeaturesInit::streamVideo(){
	
	ros::NodeHandle n;
	ros::Rate r(2); // 10 hz
	
	cv::Mat img;
	
	ros::Subscriber sub_pointcloud = n.subscribe("/pointcloud", 1, &Test_RGBFeaturesInit::get_cloud_Callback, this);
	
	r.sleep();
	ros::spinOnce();
	
	while(ros::ok()){
		
		img = extractRGBfromCloud(cloudrec);
		imshow("Video_stream",img,true);
	
		r.sleep();
		ros::spinOnce();
	}
	
}

inline void Test_RGBFeaturesInit::computeSURFfeatures(cv::Mat im , cv::Mat &desc , std::vector<cv::KeyPoint> &kp){
	cv::Mat im_gray;
		
	cv::SURF surf;
	
	cv::cvtColor(im,im_gray,CV_RGB2GRAY,1);
  
	surf(im_gray,cv::Mat(),kp);


	cv::SurfDescriptorExtractor extractor;

	extractor.compute(im_gray, kp, desc);
	
}

inline void Test_RGBFeaturesInit::imshow(char* window_name, cv::Mat im, bool block){
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE); 
	cv::imshow(window_name, im); 
	if(block)
		cv::waitKey();
}

inline void Test_RGBFeaturesInit::featureshow(char* window_name, cv::Mat im, std::vector<cv::KeyPoint> kp, bool block){
	cv::Mat im_out;
	im_out=im;
	
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE); 
	cv::drawKeypoints(im, kp, im_out, cv::Scalar(0,0,255)); //BGR
	
	cv::imshow(window_name, im_out); 
	
	if(block)
		cv::waitKey();	
}


void Test_RGBFeaturesInit::PrintCloudsToPLY(char* filename){ 
	
	long pos;
	int npoints=0;
	
	std::ofstream ply_file(filename);
	ply_file << "ply\n";
	ply_file << "format ascii 1.0\n";
	ply_file << "element vertex ";  pos=ply_file.tellp();
	ply_file << "                   \n";
	ply_file << "property float x\n";
	ply_file << "property float y\n";
	ply_file << "property float z\n";
	ply_file << "property uchar red\n";
	ply_file << "property uchar green\n";
	ply_file << "property uchar blue\n";
	ply_file << "end_header\n";

	
	for(unsigned int v=0;v<cloud_s->height;v++) {	  //480
		for(unsigned int u=0;u<cloud_s->width;u++) {   //640
			if (!isnan((*cloud_s)(u,v).z)){
				ply_file << (*cloud_s)(u,v).x << " " << (*cloud_s)(u,v).y << " " << (*cloud_s)(u,v).z << " "
						 <<(int) (*cloud_s)(u,v).r << " " << (int) (*cloud_s)(u,v).g << " " << (int) (*cloud_s)(u,v).b << "\n";
				npoints++;
			}
		}
	}
	
	Eigen::Vector3d vec;
	
	for(unsigned int v=0;v<cloud_d->height;v++) {	  //480
		for(unsigned int u=0;u<cloud_d->width;u++) {   //640
			if (!isnan((*cloud_d)(u,v).z)){
				
				vec(0) = (*cloud_d)(u,v).x;
				vec(1) = (*cloud_d)(u,v).y;
				vec(2) = (*cloud_d)(u,v).z;
				
				vec = T.topLeftCorner<3,3>()*vec + T.topRightCorner<3,1>();
				
				ply_file << vec(0) << " " << vec(1) << " " << vec(2) << " "
						 <<(int) (*cloud_d)(u,v).r << " " << (int) (*cloud_d)(u,v).g << " " << (int) (*cloud_d)(u,v).b << "\n";
				npoints++;
			}
		}
	}
	
		
	ply_file.seekp(pos);
	ply_file << npoints;
	ply_file.close();
	
}

void Test_RGBFeaturesInit::get_cloud_Callback(const PCloud& pcloud){
	cloudrec=pcloud;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Global Variables
char filename_s[100];
char filename_d[100];

//testa o meu algoritmo
void main0(int argc, char* argv[]){
	ros::init(argc, argv, "Test_Initblock");
	ros::NodeHandle n;
	
	if(argc==2){
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_s.txt", NULL);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_d.txt", NULL);
	}else{
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);	
	}

	Test_RGBFeaturesInit c(filename_s,filename_d);
	
	//test computation time
	ros::Time start = ros::Time::now();
	c.compute();
	ros::Time end = ros::Time::now();
	printf("Algorithm: %.6f seconds\n", (end-start).toSec());
	
	c.PrintCloudsToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/clouds_ORB.ply");
	
	std::cout << "done!\n";
	
}
//test o ORB vs SURF
void main1(int argc, char* argv[]){
	ros::init(argc, argv, "Test_Initblock");
	ros::NodeHandle n;
	
	if(argc==2){
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_s.txt", NULL);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_d.txt", NULL);
	}else{
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);	
	}
	
	Test_RGBFeaturesInit c1(filename_s,filename_d);
	Test_RGBFeaturesInit c2(filename_s,filename_d);
	
	
	//test computation time
	ros::Time start = ros::Time::now();
	c1.compute();
	ros::Time end = ros::Time::now();
	printf("ORB Algorithm: %.6f seconds\n", (end-start).toSec());
	
	c1.PrintCloudsToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/clouds_ORB.ply");
	
	start = ros::Time::now();
	c2.compute_surf(false);
	end = ros::Time::now();
	printf("SURF Algorithm: %.6f seconds\n", (end-start).toSec());
	
	c2.PrintCloudsToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/clouds_SURF.ply");
	
	std::cout << "done!\n";
}
//ORB debug mode
void main2(int argc, char* argv[]){
	ros::init(argc, argv, "Test_Initblock");
	ros::NodeHandle n;
	
	if(argc==2){
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_s.txt", NULL);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_d.txt", NULL);
	}else{
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);	
	}
	
	Test_RGBFeaturesInit c(filename_s,filename_d);
	
	//test computation time
	std::cout<< "Number of inliers: " << c.compute_ORB() << "\n";
	
	std::cout<< "T=\n" << c.T << "\n";
	
	c.PrintCloudsToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/clouds_ORB.ply");
	
	std::cout << "done!\n";
	
}
//Surf debug mode
void main3(int argc, char* argv[]){
	ros::init(argc, argv, "Test_Initblock");
	ros::NodeHandle n;
	
	if(argc==2){
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_s.txt", NULL);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/cloud_d.txt", NULL);
	}else{
		sprintf (filename_s, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[2]);
		sprintf (filename_d, "/home/pedrovieira/ros_workspace/icp_ros/bin/clouds_txt/%s", argv[3]);	
	}
	
	Test_RGBFeaturesInit c(filename_s,filename_d);
	
	//test computation time
	std::cout<< "Number of inliers: " << c.compute_surf(true) << "\n";
	
	std::cout<< "T=\n" << c.T << "\n";
	
	c.PrintCloudsToPLY("/home/pedrovieira/ros_workspace/icp_ros/bin/world_clouds/clouds_SURF.ply");
	
	std::cout << "done!\n";
	
}

//Make map
void main4(int argc, char* argv[]){
	ros::init(argc, argv, "Test_Initblock");
	
	Test_RGBFeaturesInit c;
	c.makemap(atoi(argv[2]));

	std::cout << "done!\n";
	
}

//Video stream
void main5(int argc, char* argv[]){
	ros::init(argc, argv, "Test_Initblock");
	
	std::cout << "start!\n";
	Test_RGBFeaturesInit c;
	c.streamVideo();
	std::cout << "done!\n";
	
}



int main(int argc, char* argv[]){
	
	if(argc<2){
		std::cout << "0 - Tempo do Algoritmo + result | cloud_s.txt cloud_d.txt\n"
				  << "1 - Tempo do Algoritmo ORB vs SURF + results | cloud_s.txt cloud_d.txt\n"
				  << "2 - ORB debug mode | cloud_s.txt cloud_d.txt\n"
				  << "3 - SURF debug mode | cloud_s.txt cloud_d.txt\n"
				  << "4 - Make map | nframes\n" 
				  << "5 - Video stream\n\n"; 
				  
		return 0;
	}
	
	
	switch(atoi(argv[1])){
	
		case 0: {main0(argc,argv);} break;
		case 1: {main1(argc,argv);} break;
		case 2: {main2(argc,argv);} break;
		case 3: {main3(argc,argv);} break;
		case 4: {main4(argc,argv);} break;
		case 5: {main5(argc,argv);} break;
	}
	
	return 0;
}


