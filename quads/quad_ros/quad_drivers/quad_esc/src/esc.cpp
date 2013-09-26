#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include "quad_esc/Esc.h"
#include <Ivy/timer.h>

quad_esc::Esc esc;
ros::Publisher esc_message;


/* 
 * Broadcast ESC information of quadrotor
 */
 
void ESCCallback (IvyClientPtr app, void* data , int argc, char **argv)
{
	
	int n = sscanf(argv[0],"%*d %*d %*d %*d %*d %*d %*d %*d %d %d %d %d %d %d %d %d",&esc.Front_rpm, &esc.Front_pwm, &esc.Back_rpm, &esc.Back_pwm, &esc.Left_rpm, &esc.Left_pwm, &esc.Right_rpm, &esc.Right_pwm);

	esc.header.stamp = ros::Time::now();

	esc_message.publish(esc);
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
	ros::init(argc, argv, "quad_esc");
	ros::NodeHandle nh("~");
	
	esc_message = nh.advertise<quad_esc::Esc>("esc", 1000);

	// Initializing Ivy
	IvyInit ("quad_status", "'Quad ESC' is READY!", 0, 0, 0, 0);
	IvyStart("127.255.255.255");
	TimerRepeatAfter (TIMER_LOOP, 500, ROSCallback, 0);
	
	// Binding Messages
	IvyBindMsg(ESCCallback, 0, "ISR_STUFF(.*)");
	
	IvyMainLoop();
	
	return 0;
}
