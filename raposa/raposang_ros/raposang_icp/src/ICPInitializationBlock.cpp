/*
 *  ICPInitializationBlock.cpp
 *  
 *  Created by Pedro Vieira on 10/23/11.
 *  
 * test: rosrun raposang_ros test_init_block 1 99 1 10000 cloud_s cloud_d
 * 
 */
#include "ICPInitializationBlock.h"

#include "ransac/RSTEstimator.h"
#include "ransac/RANSAC.h"


int RGBFeaturesInit::compute(){
	
	cv::Mat im_s = extractRGBfromCloud(cloud_s);
	cv::Mat im_d = extractRGBfromCloud(cloud_d);
	
	cv::Mat desc_s;  
	cv::Mat desc_d;  
	std::vector<cv::KeyPoint> kp_s;
	std::vector<cv::KeyPoint> kp_d;
	
	computeORBfeatures(im_s , desc_s , kp_s);
	computeORBfeatures(im_d , desc_d , kp_d);
	
	//cv::BruteForceMatcher<cv::Hamming> matcher;
	std::vector<cv::DMatch> matches;
	//matcher.match(desc_s,desc_d,matches);	
	macthpoints(desc_s,desc_d,matches);
	
	make3Dpairs(kp_s, kp_d, matches);
	
	TakeOutOutliers();

	return inliers.size();
} 

int RGBFeaturesInit::computeSURF(){
	
		
	cv::Mat im_s = extractRGBfromCloud(cloud_s);
	cv::Mat im_d = extractRGBfromCloud(cloud_d);
	
	
	cv::Mat desc_s;  
	cv::Mat desc_d;  
	std::vector<cv::KeyPoint> kp_s;
	std::vector<cv::KeyPoint> kp_d;
	
	
	computeSURFfeatures(im_s , desc_s , kp_s);
	computeSURFfeatures(im_d , desc_d , kp_d);
	
		
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(desc_s,desc_d,matches);
	
	
	make3Dpairs(kp_s, kp_d, matches);
	
	TakeOutOutliers();

	return inliers.size();
} 


cv::Mat RGBFeaturesInit::extractRGBfromCloud(const PCloud& cloud){
	
	cv::Mat RGBImage(cloud->height,cloud->width,CV_8UC3);
		
	for(unsigned int v=0;v<cloud->height;v++) {	    //480
		for(unsigned int u=0;u<cloud->width;u++) {  //640
			
			RGBImage.at<cv::Vec3b>(v,u)[0] = (int) (*cloud)(u,v).b; 
			RGBImage.at<cv::Vec3b>(v,u)[1] = (int) (*cloud)(u,v).g; 
			RGBImage.at<cv::Vec3b>(v,u)[2] = (int) (*cloud)(u,v).r; 	
		}
	}
	
	return RGBImage;
}

void RGBFeaturesInit::computeORBfeatures(const cv::Mat& im , cv::Mat &desc , std::vector<cv::KeyPoint> &kp){
	cv::Mat im_gray;

	cv::ORB orb;
	
	cv::cvtColor(im,im_gray,CV_RGB2GRAY,1);
	
	orb(im_gray,cv::Mat(),kp, desc);
}
void RGBFeaturesInit::computeSURFfeatures(const cv::Mat& im , cv::Mat &desc , std::vector<cv::KeyPoint> &kp){
	cv::Mat im_gray;
	
	cv::SURF surf;
	
	cv::cvtColor(im,im_gray,CV_RGB2GRAY,1);
  
	surf(im_gray,cv::Mat(),kp);	
	
	cv::SurfDescriptorExtractor extractor;

	extractor.compute(im_gray, kp, desc);
	

}

void RGBFeaturesInit::macthpoints(const cv::Mat& desc_s,const cv::Mat& desc_d, std::vector<cv::DMatch> &matches){
		
		int dist, d1, d2;
		cv::DMatch match; 

		matches.clear();

		int tt=0;
		for(unsigned int i = 0; i < desc_s.rows ; i++){
			
			d1=d2=0;
			
			for(unsigned int j = 0; j < desc_d.rows; j++){
				dist=0;
				
				//hamming distance
				for(int k=0;k<32;k++){
					if(desc_s.at<uchar>(i,k) == desc_d.at<uchar>(j,k))
						dist++;
				}
				
				if(dist>d1){ // if this feature matches better than current best
					d2 = d1;
					d1 = dist;
					match.queryIdx = i;
					match.trainIdx = j;
				}else{
					if(dist>d2)
						d2 = dist;
				}
			
			}
			
			tt++;
			if(d1 > 16){
				if(d1 > 2*d2){	
				//if(d2 < 0.7*d1){ 	
					// Store the change in position
					matches.push_back(match);
				}
			}
		}
		//std::cout<< "numero de features= " << tt << "   " << "good features = " << matches.size() << "\n";
		
	}

void RGBFeaturesInit::make3Dpairs(std::vector<cv::KeyPoint> kp_s, std::vector<cv::KeyPoint> kp_d, std::vector<cv::DMatch> matches){
	
	matches3D.clear();
	
	int point3d_s;
	int point3d_d;
	
	int u, v;
		
	for (int i=0; i<matches.size(); i++) 
	{
		u = kp_s[matches[i].queryIdx].pt.x;
		v = kp_s[matches[i].queryIdx].pt.y;
		
		if( isnan((*cloud_s)(u,v).z) ) continue;
		
		point3d_s = v*cloud_s->width + u;
	
		u = kp_d[matches[i].trainIdx].pt.x;
		v = kp_d[matches[i].trainIdx].pt.y;
		
		if( isnan((*cloud_d)(u,v).z) ) continue;
		
		point3d_d = v*cloud_d->width + u;
		
		matches3D.push_back(std::make_pair(point3d_s, point3d_d));
	}	
}

void RGBFeaturesInit::TakeOutOutliers(){
	//input:  matches
	//output: inliers
	
	inliers.clear();
	
	//Variaveis para o RANSAC
	std::vector<double> RSTParameters;
	RSTEstimator lpEstimator(th, cloud_s,cloud_d);
	double desiredProbabilityForNoOutliers=prob;
	
	//Aplica RANSAC
	//inliers modificado dentro da funçao
	RANSAC<std::pair<int,int>,double>::compute( 
												RSTParameters, 
												&lpEstimator, 
												matches3D,
												desiredProbabilityForNoOutliers,
												inliers);
	
	if (inliers.size()>=4) {
		T << RSTParameters[0] , RSTParameters[1] , RSTParameters[2] ,     RSTParameters[9]  ,
			 RSTParameters[3] , RSTParameters[4] , RSTParameters[5] ,     RSTParameters[10] ,
		     RSTParameters[6] , RSTParameters[7] , RSTParameters[8] ,     RSTParameters[11] ,
					0		  ,   	    0        ,        0         ,            1          ; 
		
	}else{
		T.setIdentity();
	}

}



/*
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
			
			
		//std::ofstream txt1("/home/pedrovieira/ros_workspace/raposang_ros/bin/source.txt");
		//std::ofstream txt2("/home/pedrovieira/ros_workspace/raposang_ros/bin/data.txt");
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
			
					if( (*cloud_s)(u,v).x - inliers[i].first(0)==0.0 && 
						(*cloud_s)(u,v).y - inliers[i].first(1)==0.0 &&
						(*cloud_s)(u,v).z - inliers[i].first(2)==0.0 )
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
			
					if( (*cloud_d)(u,v).x - inliers[i].second(0)==0.0 && 
						(*cloud_d)(u,v).y - inliers[i].second(1)==0.0 &&
						(*cloud_d)(u,v).z - inliers[i].second(2)==0.0 )
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


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "test_init");
	ros::NodeHandle n;
	
	std::string path_txt = "/home/pedrovieira/ros_workspace/raposang_ros/bin/clouds_txt/";
	std::string path_ply = "/home/pedrovieira/ros_workspace/raposang_ros/bin/world_clouds/";
	std::string f1,f2, fply1, fply2;
	
	f1 = path_txt + argv[5] + ".txt";
	f2 = path_txt + argv[6] + ".txt";
	
	fply1 = path_ply + argv[5] + ".ply";
	fply2 = path_ply + argv[6] + ".ply";
	
	TestINI t(f1.data() ,f2.data());
	
	double p = (double)atoi(argv[2])/100.0;
	double tr = (double)atoi(argv[3])/(double)atoi(argv[4]);
	
	t.setDesiredP(p);
	t.setRansacThreshold(tr); 
	//t.setMinMatchDist(3);
	
	int f=0;
	switch(atoi(argv[1])){
		case 0:{f=t.test_compute();}break;
		case 1:{f=t.test_algTime();}break;
	}
	
	if(f >= 4) t.Writeclouds(fply1.data(),fply2.data());

	
	return 0;
}
*/
