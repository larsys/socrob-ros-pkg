#include "ros/ros.h"
#include <sys/wait.h>

/* 
 * Open the link to quadrotor
 */

int main(int argc, char **argv)
{
	// Initializing ROS
	ros::init(argc, argv, "quad_link");
	ros::NodeHandle nh("~");
	
	system("$PAPARAZZI_HOME/sw/ground_segment/tmtc/link -s 57600 -d /dev/Zigbee_Link");
	ROS_FATAL("Error opening modem serial device...Retrying!");

  return 0;
}
