#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>

#define MILLISEC 1000.0

/* 
 * Set telemetry to 'Scaled' mode.
 * Sends a command every $PERIOD seconds.
 * Parameters:  AC_ID;
 * 				PERIOD;
 */

int ac_id, period, throttle=50;

void SettingCallback (TimerId id, void *user_data, unsigned long delta)
{
	throttle=throttle+1;
	IvySendMsg ("input2ivy RC_4CH %d 1 200 0 0 0",ac_id);
	if (!ros::ok()) {
		IvyStop();
		exit(0);
	}
}

int main(int argc, char **argv)
{
	// Initializing ROS
	ros::init(argc, argv, "quad_control");
	ros::NodeHandle nh("~");
	
	// Getting ROS Parameters
	if (!nh.getParam("ac_id", ac_id))
	{
		ROS_FATAL("Failed to get param 'AC_ID'");
		exit(0);
	}
	nh.param("period", period, 1);
	
	// Initializing Ivy
	IvyInit ("quad_control", "'Quad Control' is READY!", 0, 0, 0, 0);
	TimerRepeatAfter (TIMER_LOOP, period*MILLISEC, SettingCallback, 0);
	IvyStart("127.255.255.255");
	IvyMainLoop();
	return 0;
}
