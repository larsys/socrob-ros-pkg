/***

If you have some question about this program, please send me message.
keiya.okada(at mark)gmail.com

Copyright (C) 2011 Keiya Okada All Rights Reserved.

***/

/*

This program distinguishes message type and gets and shows file size of image.
The way to distinguish is getting parameter of image_transport (raw, compressed and theora)

To get image file, this program uses "ros::serialization::serializationLength" function.
Bellow link is documentation of this function.
" http://www.ros.org/doc/api/roscpp_serialization/html/serialization_8h_source.html "

Parameter should be another node (GUI?).
I created some functions to get file size independently.

**Caution**
This program can work but is incomplete!
We have to make this program communicate with another node to get parameter and send file size.

*/

/***********/
/* Include */
/***********/

// include from depedend packages //

#include <ros/ros.h>
//#include <ros/subscriber.h>             // Now it's not necessary
//#include <message_filters/subscriber.h> // Now it's not necessary

// includes message type //

#include "std_msgs/UInt32.h"               // is used to publish file size for external node
#include "std_msgs/String.h"               // is used to get parameter (raw, compressed and theora)
#include "sensor_msgs/Image.h"             // is used to get file size of raw image
#include "sensor_msgs/CompressedImage.h"   // is used to get file size of compressed image
#include "theora_image_transport/Packet.h" // is used to get file size of theora image

/*************/
/* Namespace */
/*************/

using namespace sensor_msgs; 
using namespace std_msgs;    // is to use UInt32 type
using namespace std;         // is to use string type

/****************/
/* Global value */
/****************/

UInt32 size;              // is message to publish file size for another node
string s;                 // is used to get parameter name (raw, compressed and theora)
ros::Publisher send_size; // declares gloval publisher to send message for another node

/*********************/
/* ParameterCallback */
/*********************/

/* 
This function shows parameter set by another node
*/

void ParameterCallback(const String::ConstPtr& param_msg)
{
 	s = param_msg->data.c_str();
 	ROS_INFO("Got param: %s", s.c_str()); // shows parameter 
}

/***********************/
/* RawFileSizeCallback */
/***********************/

/* 
This function gets, shows, and publishes file size of raw image
*/

void RawFileSizeCallback(const ImageConstPtr& raw_size_msg)
{
    ros::NodeHandle raw; // nodehandle for publishing file size of raw image
    
    size.data = ros::serialization::serializationLength(*raw_size_msg); // gets file size from message size
   	send_size = raw.advertise<UInt32&>("send_filesize", 100);           // tells the master that we are going to be publishing a message of type std_msgs/UInt32
 	ROS_INFO("filesize(msg): %d byte", size.data);  
 	send_size.publish(size);                                            // publishes file size of raw image for another node
}

/******************************/
/* CompressedFileSizeCallback */
/******************************/

/* 
This function gets, shows, and publishes file size of compressed image
*/

void ComperssedFileSizeCallback(const CompressedImageConstPtr& com_size_msg)
{
    ros::NodeHandle com; // nodehandle for publishing file size of compressed image
    
    size.data = ros::serialization::serializationLength(*com_size_msg); // gets file size from message size
   	send_size = com.advertise<UInt32&>("send_filesize", 100);           // tells the master that we are going to be publishing a message of type std_msgs/UInt32
 	ROS_INFO("filesize(msg): %d byte", size.data);  
 	send_size.publish(size);                                            // publishes file size of compressed image for another node
}

/**************************/
/* TheoraFileSizeCallback */
/**************************/

/* 
This function gets, shows, and publishes file size of theora image.
Now this function is not used.
I didn't find the difference between "PacketConstPtr&" and "Packet&" because file size were same.
*/

/*void TheoraFileSizeCallback(const theora_image_transport::PacketConstPtr& theora_size_msg)
{
    ros::NodeHandle theo; // nodehandle for publishing file size of compressed image
    
    size.data = ros::serialization::serializationLength(*theora_size_msg); // gets file size from message size
   	send_size = theo.advertise<UInt32&>("send_filesize", 100);             // tells the master that we are going to be publishing a message of type std_msgs/UInt32
 	ROS_INFO("filesize(msg): %d byte", size.data);  
 	send_size.publish(size);                                               // publishes file size of compressed image for another node
}*/

/**************************/
/* TheoraFileSizeCallback */
/**************************/

/* 
This function gets, shows, and publishes file size of theora image
I didn't find the difference between "PacketConstPtr&" and "Packet&" because file size were same.
*/

void TheoraFileSizeCallback(const theora_image_transport::Packet& theora_size_msg)
{
    ros::NodeHandle theo; // nodehandle for publishing file size of compressed image
    
    size.data = theora_size_msg.data.size();                   // gets file size from message size. This function does not use ros::serialization::serializationLength
                                                               // "data.size()" can get size of packet. Please see bellow link at L.64
                                                               // " http://ros.org/doc/electric/api/theora_image_transport/html/Packet_8h_source.html "
   	send_size = theo.advertise<UInt32&>("send_filesize", 100); // tells the master that we are going to be publishing a message of type std_msgs/UInt32
 	ROS_INFO("filesize(msg): %d byte", size.data);  
 	send_size.publish(size);                                   // publishes file size of compressed image for another node
}

/********/
/* main */
/********/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_filesize");         // declares name of node 
    ros::NodeHandle nh;                            // nodehandle to subscribe message
    ros::NodeHandle param  = ros::NodeHandle("~"); // declares parameter nodehandle
    ros::Subscriber parameter, file;               // declare two subscriber 
    
    param.getParam("image_transport", s); // puts "image_transport" parameter on string "s".
                                          // Now "image_transport" parameter is determined by typing directly when this program is started to run
                                          // **rosrun package_name get_filesize _image_transport:=compressed
                                          // --> after that, s becomes "compressed" 
    
    //parameter = nh.subscribe("compressed/param",1, ParameterCallback); // In future, parameter should be gotten from another node especially GUI 

    ////////////////////////////////////////////////////
    // This part is used to distinguish message type. //
    ////////////////////////////////////////////////////

    if(s == "raw")
        file = nh.subscribe("bb2/output", 100, RawFileSizeCallback);
    else if(s == "compressed")
     	file = nh.subscribe("bb2/output/compressed", 100, ComperssedFileSizeCallback);
    else if(s == "theora")
     	file = nh.subscribe("bb2/output/theora", 100, TheoraFileSizeCallback);

    ROS_INFO("%s", file.getTopic().c_str());	
    ros::spin();
}
