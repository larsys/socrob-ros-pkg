#include "ros/ros.h"
#include <sys/wait.h>
#include <string>
#include <iostream>

/* 
 * Open the link to quadrotor
 */

int main(int argc, char **argv)
{
	// Initializing ROS
	ros::init(argc, argv, "quad_link");
	ros::NodeHandle nh("~");
	
	std::string dev_port_;
	std::string str1, str2;
	
	nh.param<std::string>("dev_port", dev_port_, "/dev/Zigbee_Link");

	str1 = "$PAPARAZZI_HOME/sw/ground_segment/tmtc/link -s 57600 -d ";
	str2 = str1 + dev_port_;
	system(str2.c_str());
	ROS_FATAL("Error opening modem serial device...Retrying!");

  return 0;
}
