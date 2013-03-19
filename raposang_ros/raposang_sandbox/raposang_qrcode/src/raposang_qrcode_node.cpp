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
#include "decodeqr.h"

QrDecoderHandle decoder;

void callbackImg(const sensor_msgs::ImageConstPtr &img_msg) {
	
		sensor_msgs::CvBridge bridge;
	
    IplImage *camera = bridge.imgMsgToCv(img_msg, "rgb8");
    
    qr_decoder_set_image_buffer(decoder,camera);
            
		qr_decoder_decode_image(decoder,camera);

   	QrCodeHeader header;
    if(qr_decoder_get_header(decoder,&header)){
        //
        // get QR code text
        // To null terminate, a buffer size is larger than body size.
        //
        char *buf=new char[header.byte_size+1];
        qr_decoder_get_body(decoder,(unsigned char *)buf,header.byte_size+1);
        ROS_INFO("FOUND TEXT: %s\n",buf);
    }
		else
			ROS_INFO("No readings");

	
	
};

int main(int argc,char *argv[])
{
	ros::init(argc, argv, "raposang_qrcode");	
	ros::NodeHandle n;
	
	image_transport::ImageTransport it(n);		
	image_transport::Subscriber sub_Img;		
		
	sub_Img = it.subscribe("input_image", 1, callbackImg);	

	decoder=qr_decoder_open();
	
  ROS_INFO("libdecodeqr version %s\n",qr_decoder_version());

	ros::Rate rate(1);	
	
	while(ros::ok()) {	
		ros::spinOnce();
		rate.sleep();
	}    

	qr_decoder_close(decoder);

	return(0);
}
