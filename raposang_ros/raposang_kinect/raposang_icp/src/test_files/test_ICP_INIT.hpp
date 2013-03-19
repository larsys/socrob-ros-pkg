//test
#include "UtilitiesBlock.h"
#include <iostream>
#include "ICPBlock.h"
#include "highgui.h"
#include "ros/ros.h"
#include "string"
#include "fstream"

class TestINI : public RGBFeaturesInit{

public:
	TestINI(const char *filename1,const char *filename2) : RGBFeaturesInit (){
		LoadCloudFromTXT(filename1, cloud_s);
		LoadCloudFromTXT(filename2, cloud_d);
	}
	
	int test_compute(){
		
		ros::Time start, end;
		float a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0; 
		
		cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Data", CV_WINDOW_AUTOSIZE);
		cv::Mat im_gray;
					
		//test extract RGB from Cloud
		start = ros::Time::now();
		cv::Mat im_s = extractRGBfromCloud(cloud_s);
		cv::Mat im_d = extractRGBfromCloud(cloud_d);
		end = ros::Time::now();
		a1 = (end-start).toSec();
		ROS_INFO("extrac time: %f", a1/2.0);
		cv::imshow("Source", im_s);
		cv::imshow("Data", im_d);
		cv::waitKey(0);	
		
		cv::Mat desc_s;  
		cv::Mat desc_d;  
		std::vector<cv::KeyPoint> kp_s;
		std::vector<cv::KeyPoint> kp_d;
		
		//ORB
		start = ros::Time::now();
		cv::ORB orb;
		cv::cvtColor(im_s,im_gray,CV_RGB2GRAY,1);
		orb(im_gray,cv::Mat(),kp_s, desc_s);
		cv::cvtColor(im_d,im_gray,CV_RGB2GRAY,1);
		orb(im_gray,cv::Mat(),kp_d, desc_d);
		end = ros::Time::now();
		a2 = (end-start).toSec();
		ROS_INFO("ORB time (features+desc): %f , Features S D: %d %d", a2/2.0, kp_s.size(), kp_d.size());
	
		featureshow("Source", im_s, kp_s, true);	
		featureshow("Data", im_d, kp_d, true);
	
		//desc info
		ROS_INFO("flag, dims, rows, cols = %d %d %d %d",desc_s.flags,desc_s.dims,desc_s.rows,desc_s.cols);
		
		
		start = ros::Time::now();
		//cv::BruteForceMatcher<cv::Hamming> matcher;
		std::vector<cv::DMatch> matches;
		//matcher.match(desc_s,desc_d,matches);	
		macthpoints(desc_s,desc_d,matches);
		end = ros::Time::now();
		a3 = (end-start).toSec();
		ROS_INFO("matche: %f %d   %f", a3, matches.size(), (float)matches.size()/float(desc_s.rows) * 100.0 );
			
			
		//std::ofstream txt1("/home/pedrovieira/ros_workspace/icp_ros/bin/source.txt");
		//std::ofstream txt2("/home/pedrovieira/ros_workspace/icp_ros/bin/data.txt");
		//for(int q=0;q<matches.size();q++){
		//	for(int m=0;m<32;m++){
		//		txt1 << (unsigned int) desc_s.at<uchar>(matches[q].queryIdx,m) << " ";
		//	}
		//	txt1 << "\n";
		//	for(int m=0;m<32;m++){
		//		txt2 <<  (unsigned int) desc_d.at<uchar>(matches[q].trainIdx,m) << " ";
		//	}
		//	txt2 << "\n";
		//}
		//txt1.close();
		//txt2.close();
		

		cv::Mat imageMatches;
		cv::drawMatches(im_s,kp_s,im_d,kp_d,matches,imageMatches,cv::Scalar(255,0,0));
		cv::imshow("matches",imageMatches);
		cv::waitKey(0);	
		
		start = ros::Time::now();
		make3Dpairs(kp_s, kp_d, matches);
		end = ros::Time::now();
		a4 = (end-start).toSec();
		ROS_INFO("matches3d: t=%f  matches=%d   out=%f", a4, matches3D.size(),  (1.0 - (float)matches3D.size()/(float)matches.size())*100.0 );
	
		start = ros::Time::now();
		TakeOutOutliers();
		end = ros::Time::now();
		a5 = (end-start).toSec();
		ROS_INFO("Ransac: t=%f matches=%d   out=%f ", a5, inliers.size() , (1.0 - (float)inliers.size()/(float)matches3D.size())*100.0 );
		
		DisplayInliers();
		
		Eigen::FullPivLU<Eigen::Matrix4d> lu_decomp(T);
		std::cout << "Rank: " << lu_decomp.rank() << std::endl;
		
		
		return inliers.size();
	}
	int test_algTime(){
		int f;
		ros::Time start, end;
		
		start = ros::Time::now();
		f=compute();
		end = ros::Time::now();
		ROS_INFO("Algorithm time: %f", (end-start).toSec());
		return f;	
	}
	
	int test_distances(){
		compute();
		
		double mean=0.0;
		Eigen::Vector3d model_p;
		Eigen::Vector3d data_p;

	
		std::cout << "inliers size: " << inliers.size() << "\n";
		for(int i=0;i<inliers.size() ;i++){
			model_p(0) = cloud_s->points[ inliers[i].first ].x;
			model_p(1) = cloud_s->points[ inliers[i].first ].y;
			model_p(2) = cloud_s->points[ inliers[i].first ].z;
	
			data_p(0) = cloud_s->points[ inliers[i].second ].x;
			data_p(1) = cloud_s->points[ inliers[i].second ].y;
			data_p(2) = cloud_s->points[ inliers[i].second ].z;
			
			mean += (model_p - data_p).norm();
		}
		
		mean /= inliers.size();
	
		std::cout << "dist mean =  " << mean << "\n";
		
		return inliers.size(); 
	}
	
	
	
	void featureshow(char* window_name, cv::Mat im, std::vector<cv::KeyPoint> kp, bool block){
		cv::Mat im_out;
		im_out=im;
	
		cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE); 
		cv::drawKeypoints(im, kp, im_out, cv::Scalar(0,0,255)); //BGR
	
		cv::imshow(window_name, im_out); 
	
		if(block)
			cv::waitKey();	
	}
	
	void DisplayInliers(){
		
		//
		std::vector<cv::KeyPoint> kp_s;
		std::vector<cv::KeyPoint> kp_d;
		std::vector<cv::DMatch> matches;
		
		//
		for(int i=0;i<inliers.size();i++){
			
			for(int v=0;v<cloud_s->height;v++) {	//480
				for(int u=0;u<cloud_s->width;u++) { //640
					if( isnan((*cloud_s)(u,v).z) ) continue;
			
					if( (*cloud_s)(u,v).x - cloud_s->points[ inliers[i].first ].x==0.0 && 
						(*cloud_s)(u,v).y - cloud_s->points[ inliers[i].first ].y==0.0 &&
						(*cloud_s)(u,v).z - cloud_s->points[ inliers[i].first ].z==0.0 )
					{
						cv::KeyPoint kp;
						kp.pt.x= u;
						kp.pt.y= v;
						
						kp_s.push_back(kp);
						break;	
					}
			
				}
			}
					
			for(int v=0;v<cloud_d->height;v++) {	//480
				for(int u=0;u<cloud_d->width;u++) { //640
					if( isnan((*cloud_d)(u,v).z) ) continue;
			
					if( (*cloud_d)(u,v).x - cloud_d->points[ inliers[i].second ].x==0.0 && 
						(*cloud_d)(u,v).y - cloud_d->points[ inliers[i].second ].y==0.0 &&
						(*cloud_d)(u,v).z - cloud_d->points[ inliers[i].second ].z==0.0 )
					{
						cv::KeyPoint kp;
						kp.pt.x= u;
						kp.pt.y= v;
						
						kp_d.push_back(kp);
						break;	
					}
			
				}
			}
		
		}
		
		ROS_INFO("key: %d %d ", kp_s.size(), kp_d.size());
		
		cv::Mat im_s = extractRGBfromCloud(cloud_s);
		cv::Mat im_d = extractRGBfromCloud(cloud_d);
		
		featureshow("Source", im_s, kp_s, true);
		featureshow("Data", im_d, kp_d, true);
		
		for(int i=0;i<kp_s.size();i++){
			cv::DMatch dd;
			
			dd.trainIdx = i;
			dd.queryIdx = i;
			
			matches.push_back(dd);
		}
		
		
		cv::Mat imageMatches;
		cv::drawMatches(im_s,kp_s,im_d,kp_d,matches,imageMatches,cv::Scalar(255,0,0));
		cv::imshow("matches",imageMatches);
		cv::waitKey(0);
		 
		
		
	}

	void Writeclouds(const char* fs_ply, const char* fd_ply){
		PrintCloudToPLY(fs_ply, cloud_s);
		
		for(unsigned int v=0;v<cloud_d->height;v++) {	 //480
			for(unsigned int u=0;u<cloud_d->width;u++) {  //640
				if (!isnan((*cloud_d)(u,v).z)){
					double x= (*cloud_d)(u,v).x;
					double y= (*cloud_d)(u,v).y;
					double z= (*cloud_d)(u,v).z;
					
					(*cloud_d)(u,v).x = x*T(0,0) + y*T(0,1) + z*T(0,2) + T(0,3);
					(*cloud_d)(u,v).y = x*T(1,0) + y*T(1,1) + z*T(1,2) + T(1,3);
					(*cloud_d)(u,v).z = x*T(2,0) + y*T(2,1) + z*T(2,2) + T(2,3);	
				}
			}
		}
		
		PrintCloudToPLY(fd_ply, cloud_d);
		
	}

};


