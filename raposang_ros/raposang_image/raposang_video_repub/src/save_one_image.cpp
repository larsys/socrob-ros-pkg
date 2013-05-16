/////////////////////////////////////////////////////////////////////////
//
// webcam.cpp --a part of libdecodeqr
//
// Copyright(C) 2007 NISHI Takao <zophos@koka-in.org>
//                   JMA  (Japan Medical Association)
//                   NaCl (Network Applied Communication Laboratory Ltd.)
//
// This is free software with ABSOLUTELY NO WARRANTY.
// You can redistribute and/or modify it under the terms of LGPL.
//
// $Id$
//

#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <stdio.h>
#include <string.h>
#include <highgui.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

bool limit = true;

void callbackImg(const sensor_msgs::ImageConstPtr &img_msg) {
	
		sensor_msgs::CvBridge bridge;

		// ROS_INFO("Got an image! \n");
	
    IplImage *camera = bridge.imgMsgToCv(img_msg, "bgr8");
    
   // cvNamedWindow("Snapshot",  0);
   // cvShowImage("Snapshot",  camera);

    char filename[1024];

    for (int i=1 ; true ; i++) {
      struct stat s;
      sprintf(filename, "newest_%03d.jpg", i);
      if (0!=stat(filename, &s)) {
	printf("%s\n", filename);
	cvSaveImage(filename, camera);
	break;
      }
    }
    
    limit = false;
	
};

int main(int argc,char *argv[])
{
	ros::init(argc, argv, "save_one_image");	
	ros::NodeHandle n;
	
	image_transport::ImageTransport it(n);		
	image_transport::Subscriber sub_Img;		
		
	sub_Img = it.subscribe("input_image", 1, callbackImg);	

	// ROS_INFO("Waiting for one image... \n");

	while(ros::ok() && limit)
		ros::spinOnce();  

 // while(ros::ok() && (cvWaitKey(5)&0xff) != 'q') {}

//	cvDestroyWindow("Snapshot");

	return(0);
}
