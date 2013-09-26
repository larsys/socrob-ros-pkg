#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include "quad_can_driver/Thrust.h"
#include <Ivy/timer.h>

quad_can_driver::Thrust thrust_msg;
ros::Publisher fp_message;

/* 
 * Broadcast Thrust information of quadrotor
 */
 
void FPCallback (IvyClientPtr app, void* data , int argc, char **argv){
    double thrust;
	
    sscanf(argv[0],"%*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %*d %lf %*d",&thrust);

    thrust_msg.header.stamp = ros::Time::now();
    thrust_msg.thrust=thrust;
    ROS_INFO("RCThrust: %f",thrust);

    fp_message.publish(thrust_msg);
}

void ROSCallback (TimerId id, void *user_data, unsigned long delta){
	if (!ros::ok()){
		IvyStop();
		exit(0);
	}
}

int main(int argc, char **argv){
	
	// Initializing ROS
    ros::init(argc, argv, "quad_fp");
	ros::NodeHandle nh("~");
	
    fp_message = nh.advertise<quad_can_driver::Thrust>("RCThrust", 1000);

	// Initializing Ivy
    IvyInit ("quad_fp", "'Quad FP' is READY!", 0, 0, 0, 0);
	IvyStart("127.255.255.255");
	TimerRepeatAfter (TIMER_LOOP, 500, ROSCallback, 0);
	
	// Binding Messages
    IvyBindMsg(FPCallback, 0, "ROTORCRAFT_FP(.*)");
	
	IvyMainLoop();
	
	return 0;
}
