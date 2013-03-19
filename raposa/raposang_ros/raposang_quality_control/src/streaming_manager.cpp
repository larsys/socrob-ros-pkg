/***

If you have some question about this program, please send me message.
keiya.okada(at mark)gmail.com

Copyright (C) 2011 Keiya Okada All Rights Reserved.

***/

/*

This program sends dummy message for streaming_prove.cpp to measure RTT between RAPOSA and teleop PC
Message type is Time32 and message includes current time as nano sec.

!!THIS PROGRAM SHOULD BE PUT ON RAPOSA ANS RAUNCH WHEN RAPOSA STARTS!!

*/

/***********/
/* Include */
/***********/

// include from depedend packages //

#include <ros/ros.h>
#include <ros/time.h> // to get current time as ROS time. " http://www.ros.org/wiki/roscpp/Overview/Time "

// includes message type //

#include "std_msgs/Time.h" // is used to send current time
#include "std_msgs/Int32.h" // is used to send current time

/*************/
/* Namespace */
/*************/

using namespace std_msgs; // shortens "std_msgs::Int32"

/********/
/* main */
/********/

int main(int argc, char **argv)
{
	
	Time timenow;        
    
	ros::init(argc, argv, "streaming_manager"); // declares name of node
    ros::NodeHandle nh_send;                    // nodehandle for this program

  	ros::Publisher request_pub = nh_send.advertise<Time>("request", 1); // tells the master that we are going to be publishing a message of type std_msgs/Int32
    ros::Rate loop_rate(1);                                              // define the publishing frequency. In this case, message is published every 1 sec. 
    
  	while(ros::ok()){                               // publishing part
  	
	    timenow.data = ros::Time::now();
        request_pub.publish(timenow);               // publishes message for streaming_prove.cpp  	
    	ros::spinOnce();                            // in this case, "ros::spin()" is not useful because we want to send current time every 1 sec
    	loop_rate.sleep();    	                    // is used to send message every 1 sec
	}
}

