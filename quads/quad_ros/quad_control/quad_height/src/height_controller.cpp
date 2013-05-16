#include "ros/ros.h"
#include "quad_can_driver/Thrust.h"
#include "mav_msgs/Height.h"
#include "quad_height/pid_height.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <math.h>

#define Ref_Rate 0.005 //rate=0.05 m/s (Ts=0.2)

double desired_height, current_height, last_height, aux_height;
btScalar yaw, pitch, roll;

void desired_heightCallback(std_msgs::Float32 const &msg){
	desired_height=msg.data;
}

void imuCallback(sensor_msgs::Imu const &imu){

	tf::Quaternion orientation;
	tf::quaternionMsgToTF(imu.orientation, orientation);
  	tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
}

void heightCallback(const mav_msgs::Height& msg){
	last_height = current_height;
	current_height = msg.height;
}

/*
* Returns the sign of a double
* 1        if the value is positive
* 0        if the value is 0
* -1       if the value is negative
*/
int sign(double v){
       return v>0?1:(v<0?-1:0);
}

double Ramp(double ref_u, double uref_old){
	
	if(fabs(ref_u-uref_old)> Ref_Rate)
		ref_u = uref_old + sign(ref_u-uref_old)*Ref_Rate;
		
	return ref_u;
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "quad_height");
	ros::NodeHandle n, nl("~");
  
	ros::Subscriber sub1 = n.subscribe("/quad_height/height_to_base", 10, heightCallback);
	ros::Subscriber sub2 = n.subscribe("/imu/data", 10, &imuCallback);
	ros::Subscriber sub3 = n.subscribe("/desired_height", 10, &desired_heightCallback);
	
	ros::Publisher thrust_pub = n.advertise<quad_can_driver::Thrust>("ThrustCtrl", 10);
  
	quad_can_driver::Thrust thrust_msg;

	ros::Rate loop_rate(10);

	ros::Time last_time, current_time;
	ros::Duration t;
	
	int cmd;
	double height_gain[5];
	double Mass;
  
	nl.param("height_p", height_gain[P_GAIN], 1.0); //0.8 0.01 0.3
	nl.param("height_i", height_gain[I_GAIN], 0.01);
	nl.param("height_d", height_gain[D_GAIN], 1.0);
	nl.param("height_imax", height_gain[I_MAX], 0.5);
	nl.param("height_imin", height_gain[I_MIN], 0.0);
	nl.param("mass",Mass,1.20);
	nl.param("desired_height",desired_height,0.0);
 
	ROS_INFO("P:%f",height_gain[P_GAIN]);
 
  	//if (!n.getParam("desired_height", desired_height)){ ROS_FATAL("Failed to get param 'desired_height'"); exit(0); }

	last_time = ros::Time::now();
	current_time = ros::Time::now();
    
	while (ros::ok()){
		
		last_time = current_time;
		current_time = ros::Time::now();
		t = current_time - last_time;
		
		aux_height=Ramp(desired_height,aux_height);
		ROS_WARN("%f",aux_height);

		cmd = PID_height(aux_height, current_height, last_height, height_gain, t.toSec(), Mass, roll, pitch);
			
		// Send message to quad_can_interface
		thrust_msg.header.stamp = ros::Time::now();
		thrust_msg.thrust = cmd;

		//ROS_INFO("%.0f", thrust_msg.thrust);

		thrust_pub.publish(thrust_msg);

		//rosspin and rate
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
