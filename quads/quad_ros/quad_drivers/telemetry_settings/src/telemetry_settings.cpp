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
 * Turn off Care-Free.
 * Set Modes 1/2 to MANUAL
 * Sends a command every $PERIOD seconds.
 * Parameters:  AC_ID;
 * 				PERIOD; 				
 */

int ac_id, period;

void SettingCallback (TimerId id, void *user_data, unsigned long delta)
{
	IvySendMsg ("ground_dl SETTING 0 %d 0",ac_id); // Telemetry -> Default
	IvySendMsg ("ground_dl SETTING 1 %d 3",ac_id); // Mode2 -> AP_MODE_ATTITUDE_DIRECT
	IvySendMsg ("ground_dl SETTING 2 %d 3",ac_id); // Mode1 -> AP_MODE_ATTITUDE_DIRECT
	IvySendMsg ("ground_dl SETTING 65 %d 0",ac_id); // Care_Free -> OFF
	
	if (!ros::ok()) {
		IvyStop();
		exit(0);
	}
}

int main(int argc, char **argv)
{
	// Initializing ROS
	ros::init(argc, argv, "telemetry_settings");
	ros::NodeHandle nh("~");
	
	// Getting ROS Parameters
	if (!nh.getParam("ac_id", ac_id))
	{
		ROS_FATAL("Failed to get param 'AC_ID'");
		exit(0);
	}
	nh.param("period", period, 1);
	
	// Initializing Ivy
	IvyInit ("telemetry_settings", "'Telemetry Settings' is READY!", 0, 0, 0, 0);
	TimerRepeatAfter (TIMER_LOOP, period*MILLISEC, SettingCallback, 0);
	IvyStart("127.255.255.255");
	IvyMainLoop();
	return 0;
}
