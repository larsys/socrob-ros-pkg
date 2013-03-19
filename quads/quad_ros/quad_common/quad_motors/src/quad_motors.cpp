#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include "std_srvs/Empty.h"
#include <Ivy/timer.h>

/* 
 * Service node to Start and Stop motors.
 * Parameters:  AC_ID; 				
 */

int ac_id;
int i;

bool STARTCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	ROS_INFO("Motors ON!");
	//IvySendMsg ("dl DL_SETTING %d 3 0",ac_id); //TO be read by paparazzi server
	IvySendMsg ("ground_dl SETTING 3 %d 0.0",ac_id);
	sleep(4);
	return true;
}

bool STOPCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	ROS_INFO("Motors OFF!");
	//IvySendMsg ("dl DL_SETTING %d 3 1",ac_id); //TO be read by paparazzi server
	IvySendMsg ("ground_dl SETTING 3 %d 1.0",ac_id);
	return true;
}

void ROSCallback (TimerId id, void *user_data, unsigned long delta) 
{ 
	ros::spinOnce();
	if (!ros::ok()){
		for (i = 0; i < 5; i++) IvySendMsg ("ground_dl SETTING 3 %d 1.0",ac_id);
		//IvySendMsg ("dl DL_SETTING %d 3 1",ac_id); //TO be read by paparazzi server
		IvyStop();
		exit(0);
	}
}

int main(int argc, char **argv)
{
	// Initializing ROS
	ros::init(argc, argv, "quad_motors");
	ros::NodeHandle nh("~");
	
	// Getting ROS Parameters
	if (!nh.getParam("ac_id", ac_id))
	{
		ROS_FATAL("Failed to get param 'AC_ID'");
		exit(0);
	}
	
	// Initializing Ivy
	IvyInit ("quad_motors", "'Quad Motors' is READY!", 0, 0, 0, 0);
	IvyStart("127.255.255.255");
	
	// Initializing ROS Services
	ros::ServiceServer service_on = nh.advertiseService("start", STARTCallback);
	ros::ServiceServer service_off = nh.advertiseService("stop", STOPCallback);
	TimerRepeatAfter (TIMER_LOOP, 1, ROSCallback, 0);
	
	//ros::AsyncSpinner spinner(0);
	//spinner.start();
	
	IvyMainLoop();
	
	return 0;
}
