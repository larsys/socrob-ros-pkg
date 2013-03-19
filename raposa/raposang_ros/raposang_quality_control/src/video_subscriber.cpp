/***

If you have some question about this program, please send me message.
keiya.okada(at mark)gmail.com

Copyright (C) 2011 Keiya Okada All Rights Reserved.

***/

/*

This program is to show the raw, compressed or theora image from RAPOSA camera.
If you want to show compressed or theora image, you have to run repulisher of image_transport package.
In that case, please type bellow code in terminal (Please see this link; " http://rm.isr.ist.utl.pt/projects/raposa/wiki/RAPOSA_Cookbook "), for example;

    **rosrun image_transport republish compressed in:=/bb2/right_for_camera raw out:=/get/image

This program also calculates FPS and gets RTT time from parameter_adjuster.cpp

In this program, some functions from supported by OpenCV are used.
If you want to see the references, please see this link; " http://opencv.willowgarage.com/documentation/c/user_interface.html "

*/

/***********/
/* Include */
/***********/

// setting for usign queue //

#include <iostream>
#include <queue>

// include from depedend packages //

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

// includes message type //

#include "std_msgs/Float32.h"   // is used to get roung trip time
#include "sensor_msgs/Image.h"  // is used to get Image message

/**********/
/* Define */
/**********/

#define frame_number 1 // maximal denominator of FPS. It is also length of queue.

/*************/
/* Namespace */
/*************/

using namespace std; // namespace to use queue

/****************/
/* Global value */
/****************/

struct timeval start, end; // calculates FPS.
queue<float> qu;           // value of queue
int count_frame;           // the total number of frames of FPS
float fps;                 // frame par second; actual time spend on showing one frame of image
int quality_right;         // image quality (in this case, jpeg_quality ratio) of right camera of bumblebee camera
int quality_left;          // image quality (in this case, jpeg_quality ratio) of left camera of bumblebee camera 

/*************************/
/* calculate average fps */
/*************************/

/* 
This function is to calculate FPS with moving average concept.
Data type of function is "double" 
Parameters are time before and after the part which shows image.
*/

double calculate_fps(struct timeval st, struct timeval en){
    
    if(count_frame < frame_number){                                                       // calculates average FPS for first some frames; 
                                                                                          // the total number of frames of FPS is smaller than defined number 
        fps += 1/((en.tv_sec - st.tv_sec) + ((en.tv_usec - st.tv_usec) / 1000000.0));     // calculates total value of actual time to show image
        qu.push( 1/((en.tv_sec - st.tv_sec) + ((en.tv_usec - st.tv_usec) / 1000000.0)) ); // pushes FPS on queue
        count_frame++;                                                                    // increases the total number of frames 
        
    }
    
    else{                                                                                               // calculates average FPS for total number of frames;  
        fps = fps - qu.front() + 1/((en.tv_sec - st.tv_sec) + ((en.tv_usec - st.tv_usec) / 1000000.0)); // meaning is; total value - oldest actual time + latest actual time
        qu.pop();                                                                                       // trashes oldest FPS  
        qu.push( 1/((en.tv_sec - st.tv_sec) + ((en.tv_usec - st.tv_usec) / 1000000.0)) );               // pushes FPS on queue
        
    }
    return ( fps / (float)count_frame ); // return average FPS
}

/*****************/
/* imageCallback */
/*****************/

/* 
This function is to get message from publisher and show the image from republisher of RAPOSA
Please see this link; " http://www.ros.org/wiki/image_transport/Tutorials/SubscribingToImages "
*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    float average_fps = 0.0;      // declares average FPS and formats it
    sensor_msgs::CvBridge bridge; // declares CvBridge from " http://www.ros.org/wiki/cv_bridge "
    ros::NodeHandle parameter_nh; // nodehandle for getting compressed ratio
    
	try
	{
        start = end; // "start" becomes "end" which is calls in "main" function.
        
        cvShowImage("RAPOSA cam", bridge.imgMsgToCv(msg, "bgr8")); // converts ros message into image      
        gettimeofday( &end, NULL );                                // gets the time as "end"
        average_fps = calculate_fps(start, end);                   // throw times before and after showing image

        parameter_nh.getParam("/bb2/right_for_video/compressed/jpeg_quality", quality_right); // gets parameter (jpeg_compressed ratio) from topic of right camera
        parameter_nh.getParam("/bb2/left_for_video/compressed/jpeg_quality", quality_left);   // gets parameter (jpeg_compressed ratio) from topic of left camera

      	ROS_INFO("number_of_frame:%d  average_fps: %9.6f quality of right: %d quality of left: %d", count_frame, average_fps, quality_right, quality_left);  
    }

  	catch (sensor_msgs::CvBridgeException& e) // exception handling
  	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}

/***************/
/* rttCallback */
/***************/

/* 
This function is to get message from publisher and show the image from RAPOSA
Please see this link " http://www.ros.org/wiki/image_transport/Tutorials/SubscribingToImages "
*/

void rttCallback(const std_msgs::Float32ConstPtr& get_rtt)
{
    ROS_INFO("rtt : %f ms", get_rtt->data); 
}

/********/
/* main */
/********/

int main(int argc, char **argv)
{
    count_frame = 0; // formats global value, (int)count_frame
    fps = 0.0;       // formats global value, (float)fps

	ros::init(argc, argv, "show_video"); // declares name of node 
    ros::NodeHandle nh;                  // nodehandle for this program
    
    cvNamedWindow("RAPOSA cam", 0);       // creates the window for camera image 
  	cvMoveWindow("RAPOSA cam", 100, 100); // adjusts the position of window
  	cvStartWindowThread();                // starts the window thread
  	
    ros::Subscriber sub_image = nh.subscribe("get/image", 100, imageCallback, ros::TransportHints().udp()); // subscribes image from republisher with UDP (I'm not sure...)
    ros::Subscriber sub_rtt = nh.subscribe("average_rtt", 1, rttCallback, ros::TransportHints().udp());     // subscribes average RTT from parameter_adjuster.cpp

    gettimeofday( &end, NULL ); // gets time as before showing image 
	ros::spin();                // ros spin to use subscribers
	cvDestroyAllWindows();      // breaks the window for camera image
}
