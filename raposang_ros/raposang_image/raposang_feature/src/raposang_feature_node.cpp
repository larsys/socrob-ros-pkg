#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <time.h>

#include <ros/ros.h>
#include <iostream>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <opencv2/gpu/gpumat.hpp> 
#include <opencv2/gpu/gpu.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <raposang_msgs/RaposaStereo.h>
#include <raposang_msgs/RaposaFeatures.h>

struct params {
	int reads_per_second;		
	bool show_window;
	int nr_features_per_frame;
	
};

using namespace cv;
using namespace std;

// GLOBALS

struct params p;

Mat imleft, imright;
ros::Publisher pub;

int nr_mask_row = 3;
int nr_mask_col = 3;

void RAPOSA_feature_extract(const raposang_msgs::RaposaStereo &msg) {
	
	int i, j, k, kl, kr, ll, lr, total_l, total_r;
	
	int * left_mono;
	int * right_mono;
	
	cv::ORB orb(p.nr_features_per_frame/(nr_mask_row*nr_mask_col));	
	
	ros::Time prev, now;
	
	prev = ros::Time::now();
	
	raposang_msgs::RaposaFeatures raposa_feat;
	
	cv_bridge::CvImageConstPtr cv_left_ptr = cv_bridge::toCvCopy(msg.left, sensor_msgs::image_encodings::MONO8);
	cv_bridge::CvImageConstPtr cv_right_ptr = cv_bridge::toCvCopy(msg.right, sensor_msgs::image_encodings::MONO8);	
	
	Mat d_left_orb, d_left_orb_mask, d_right_orb;
	vector<KeyPoint>  k_left_orb, k_right_orb, kaux_left_orb, kaux_right_orb;
	vector<DMatch> truematches;
	vector<vector<DMatch> > matches;
	
	int r_step_row = 480/nr_mask_row, r_step_col = 640/nr_mask_col;		
	int i0, i1, j0, j1;
	
	for(i=0; i<nr_mask_col; i++) {
		i0 = i * r_step_col;
		i1 = i0 + r_step_col;
		for(j=0; j<nr_mask_row; j++) {
			j0 = j * r_step_row;
			j1 = j0 + r_step_row;
			orb(cv_left_ptr->image(Range(j0, j1), Range(i0, i1)), Mat(), kaux_left_orb);
			orb(cv_right_ptr->image(Range(j0, j1), Range(i0, i1)), Mat(), kaux_right_orb);
			
			for(k=0; k<kaux_left_orb.size(); k++){
				kaux_left_orb[k].pt.x += i0;
				kaux_left_orb[k].pt.y += j0;
			}
			for(k=0; k<kaux_right_orb.size(); k++){
				kaux_right_orb[k].pt.x += i0;
				kaux_right_orb[k].pt.y += j0;
			}
			
			k_left_orb.insert(k_left_orb.end(), kaux_left_orb.begin(), kaux_left_orb.end());
			k_right_orb.insert(k_right_orb.end(), kaux_right_orb.begin(), kaux_right_orb.end());
		}
	}	

	orb(cv_left_ptr->image,  Mat(), k_left_orb,  d_left_orb,  true);	
	orb(cv_right_ptr->image, Mat(), k_right_orb, d_right_orb, true);	
	
	
	left_mono  = (int *) calloc(k_left_orb.size(), sizeof(int));
	right_mono = (int *) calloc(k_right_orb.size(), sizeof(int));	

	//ROS_INFO("NR KEY LEFT: %d RIGHT: %d", k_left_orb.size(), k_right_orb.size());

	BruteForceMatcher<Hamming> matcher;

	if ((k_left_orb.size() > 0) && (k_right_orb.size() > 0)) {

		matcher.knnMatch(d_left_orb, d_right_orb, matches, 2);

		float ratio = 0.7f;
		for(i=0; i < matches.size(); i++) {
			
			if(matches[i][0].distance < matches[i][1].distance*ratio) {
				
				kl = matches[i][0].queryIdx;			
				kr = matches[i][0].trainIdx;

				if (((k_left_orb[kl].pt.y - k_right_orb[kr].pt.y) < 2.0) && (k_left_orb[kl].pt.y - k_right_orb[kr].pt.y) > -2.0)		
					truematches.push_back(matches[i][0]);

			}
		}
		
		raposa_feat.stereo_features.resize(truematches.size());
		
		for(i=0; i < truematches.size(); i++) {		
			
			kl = truematches[i].queryIdx;			
			kr = truematches[i].trainIdx;		
				
			left_mono[kl] = 1;
			right_mono[kr] = 1;
														
			raposa_feat.stereo_features[i].u  = (k_left_orb[kl].pt.y + k_right_orb[kr].pt.y) / 2.0; 									
			raposa_feat.stereo_features[i].vl = k_left_orb[kl].pt.x;
			raposa_feat.stereo_features[i].vr = k_right_orb[kr].pt.x;	
			raposa_feat.stereo_features[i].response = (k_left_orb[kl].response + k_right_orb[kr].response) / 2.0; 
						
			for(j=0; j<32; j++) 
				raposa_feat.stereo_features[i].descriptor[j] = d_left_orb.at<unsigned char>(kl,j);				
						 			
		}
		
		total_l = k_left_orb.size();
		for(i=0; i< k_left_orb.size(); i++)
			total_l -= left_mono[i];
			
		total_r = k_right_orb.size();
		for(i=0; i< k_right_orb.size(); i++)
			total_r -= right_mono[i];	
				
		raposa_feat.left_features.resize(total_l);
		raposa_feat.right_features.resize(total_r);
		
		ll=0;
		lr=0;		
		
		for(i=0; i < k_left_orb.size(); i++) {
			
			if(left_mono[i] == 0) {
														
				raposa_feat.left_features[ll].u = k_left_orb[i].pt.y; 									
				raposa_feat.left_features[ll].v = k_left_orb[i].pt.x;
				raposa_feat.left_features[ll].response = k_left_orb[i].response; 
							
				for(j=0; j<32; j++) 
					raposa_feat.left_features[ll].descriptor[j] = d_left_orb.at<unsigned char>(i,j);				
							
				ll++;
			}
		}

		for(i=0; i < k_right_orb.size(); i++) {
				
			if(right_mono[i] == 0) {
														
				raposa_feat.right_features[lr].u = k_right_orb[i].pt.y; 									
				raposa_feat.right_features[lr].v = k_right_orb[i].pt.x;
				raposa_feat.right_features[lr].response = k_right_orb[i].response; 
							
				for(j=0; j<32; j++) 
					raposa_feat.right_features[lr].descriptor[j] = d_right_orb.at<unsigned char>(i,j);				
				
				lr++;
			}			
		}
		
		pub.publish(raposa_feat);
		
	   		
		if (p.show_window) {
			Mat imageMatches;
			drawMatches(cv_left_ptr->image, k_left_orb, cv_right_ptr->image, k_right_orb, truematches, imageMatches, Scalar(0,255,0), Scalar(0,0,255));

			imshow("Matched", imageMatches);

			cv::waitKey(3); 
		}
	}
	
	//ROS_INFO(" ");
	
	free(left_mono);
	free(right_mono);
}

int main(int argc, char **argv) {

	int i, j;

	ros::init(argc, argv, "raposang_feature");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
		
	ros::Subscriber sub = n.subscribe("input_stereo", 1, RAPOSA_feature_extract);
	pub = n.advertise<raposang_msgs::RaposaFeatures>("output", 1);
	
	nh.param("reads_per_second", p.reads_per_second, 15);
	nh.param("show_window", p.show_window, true);
	nh.param("nr_features_per_frame", p.nr_features_per_frame, 500);

	nh.param("nr_tiles_height", nr_mask_row, 1);
	nh.param("nr_tiles_width",  nr_mask_col, 1);
			
	/*mask.assign(nr_mask_col * nr_mask_row, Mat::zeros(480, 640, CV_8UC1));
	
	int r_step_row = 480/nr_mask_row;
	int r_step_col = 640/nr_mask_col;	
	
	for(i=0; i<nr_mask_row; i++) {
		for(j=0; j<nr_mask_col; j++) {
			ROS_INFO("SIZE: %d %d", mask[i*3+j].rows, mask[i*3+j].cols);
			ROS_INFO("RECT: %d %d %d %d", i*r_step_row, 
							j*r_step_col, 
							r_step_row, 
							r_step_col); 		
			Rect r(	j*r_step_col, 
							i*r_step_row,
							r_step_col, 
							r_step_row); 
							
			Mat roi(mask[i*3+j], r);
			roi = Scalar(255);
		}
	}*/
		
	ros::Rate rate(p.reads_per_second);	

	if (p.show_window)
		namedWindow("Matched");
	
	while(ros::ok()) {	
		ros::spinOnce();
		rate.sleep();
	}
	 
	return 0;

}
