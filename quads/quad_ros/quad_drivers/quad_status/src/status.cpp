#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include "quad_status/Status.h"
#include <Ivy/timer.h>

quad_status::Status status;
ros::Publisher status_message;
float batt_limit=11.0;

/* 
 * Broadcast Status information of quadrotor
 */
 
void STATCallback (IvyClientPtr app, void* data , int argc, char **argv)
{
    double run_time, vsupply;
    int rc_status, motors_on, exctrl_mode;

    sscanf(argv[0],"%*d %*d %*d %d %*d %*d %*d %*d %*d %d %*d %*d %*d %d %lf %lf",&rc_status, &motors_on, &exctrl_mode, &vsupply, &run_time);

    status.header.stamp = ros::Time::now();
	
    if(rc_status==0) status.RC_OK=true;
    else status.RC_OK=false;

    if(motors_on==0) status.Motors_ON=false;
    else status.Motors_ON=true;

    if(exctrl_mode==1)	status.ExCtrl_ON=true;
    else status.ExCtrl_ON=false;

    status.Batt_Voltage=vsupply/10;
    if (status.Batt_Voltage<=batt_limit) ROS_WARN("Battery Level at %fV!!!",status.Batt_Voltage);

    status.Run_Time=ros::Duration(run_time);

    status_message.publish(status);
}

void ROSCallback (TimerId id, void *user_data, unsigned long delta) 
{ 
	if (!ros::ok()){
		IvyStop();
		exit(0);
	}
}

int main(int argc, char **argv)
{
	
	// Initializing ROS
    ros::init(argc, argv, "quad_status");
	ros::NodeHandle nh("~");
	
    status_message = nh.advertise<quad_status::Status>("status", 1000);

	// Initializing Ivy
    IvyInit ("quad_status", "'Quad Status' is READY!", 0, 0, 0, 0);
	IvyStart("127.255.255.255");
	TimerRepeatAfter (TIMER_LOOP, 500, ROSCallback, 0);
	
	// Binding Messages
    IvyBindMsg(STATCallback, 0, "ROTORCRAFT_STATUS(.*)");
	
	IvyMainLoop();
	
	return 0;
}
