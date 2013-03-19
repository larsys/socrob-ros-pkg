/***

If you have some question about this program, please send me message.
keiya.okada(at mark)gmail.com

Copyright (C) 2011 Keiya Okada All Rights Reserved.

***/

/*

This program receives dummy message from streaming_manager.cpp to measure RTT between RAPOSA and teleop PC
This program also sends time difference for parameter_adjuster.cpp to adjust compressed ratio automatically
Message type is Time and message includes current time as nano sec.

*/

/***********/
/* Include */
/***********/

// include from depedend packages //

#include <ros/ros.h>
#include <ros/time.h> // to get current time as ROS time. " http://www.ros.org/wiki/roscpp/Overview/Time "

// includes message type //

#include "std_msgs/Time.h"
#include "std_msgs/Int32.h" // is used to send the time difference

/*************/
/* Namespace */
/*************/

using namespace std_msgs; // shortens "std_msgs::Int32"

/****************/
/* Global value */
/****************/

ros::Publisher response_pub; // declares gloval publisher to send message for parameter_adjuster.cpp
Time getting_time;          // this value is used in "request_received" only but I put it on here because I wanted to remove extra time
                             // If it is declared inside of function, it takes a little more time

/********************/ 
/* request_received */
/********************/

/* 
This function receives dummy message from streaming_manager.cpp
Dummy message includes sent time at streaming_manager.cpp
This function also publishes message including sent time at streaming_manager.cpp
*/

void request_received(const TimeConstPtr& get_request)
{
// 	getting_time.data = get_request->data; // time sent by streaming_manager.cpp is put on "getting_time" message to send for parameter_adjuster.cpp
    response_pub.publish(get_request);    // publishes "getting_time" message for parameter_adjuster.cpp
}

/********/
/* main */
/********/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "streaming_prove"); // declares name of node
    ros::NodeHandle nh;                       // nodehandle for this program

	response_pub = nh.advertise<Time>("response", 1);                          // tells the master that we are going to be publishing a message of type std_msgs/Int32
    ros::Subscriber get_request= nh.subscribe("request", 1, request_received); // subscribes dummy message from streaming_manager.cpp
	ros::spin();                                                               // ros spin to use subscribers
}
