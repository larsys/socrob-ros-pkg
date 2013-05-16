#include "ros/ros.h"
#include <math.h>
#include "std_msgs/String.h"
#include "quad_can_driver/serial_interface.h"
#include "quad_can_driver/Thrust.h"
#include "quad_can_driver/Attitude.h"

#define PI 3.14159265359

struct str_ExternControl ExternControl;

void ThrustCallback(const quad_can_driver::Thrust& msg){
	if(msg.thrust>255){
		ExternControl.Throttle = 255;
	}else if(msg.thrust<0){
		ExternControl.Throttle = 0;
	}else{
		ExternControl.Throttle = msg.thrust;
	}
}

void ATTCallback(const quad_can_driver::Attitude& msg){
	
	float att_coef = 128/(PI/4);
	float roll_, pitch_;
	
	pitch_ = round(-msg.pitch*att_coef);
	roll_ = round(-msg.roll*att_coef);
	
	if(pitch_>127){
		pitch_ = 127;
	}else if(pitch_<-128){
		pitch_ = -128;
	}
	
	if(roll_>127){
		roll_ = 127;
	}else if(roll_<-128){
		roll_ = -128;
	}
	
	ExternControl.Pitch = pitch_;
	ExternControl.Roll = roll_;
	
	//Falta o yaw (saber a que angulos correspondem as entradas)
	
	ROS_INFO("I'll send: [%d] Roll & [%d] Pitch", ExternControl.Roll, ExternControl.Pitch);
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "can_interface");
	ros::NodeHandle nh;
	  
	ros::Subscriber sub1 = nh.subscribe("ThrustCtrl", 10, ThrustCallback);
	ros::Subscriber sub2 = nh.subscribe("AttitudeCtrl", 10, ATTCallback);

	double freq_ = 500.0; //Hz - send msg periodically (at least once every second)
	std::string port_ = "/dev/ttyUSB1";
	int speed_ = 115200;
	
	if (freq_<1){
		ROS_FATAL("Frequency must be greater than 1Hz");
		return 0;
	}
		
	ros::Rate loop_rate(freq_);
	
	InitSerialInterface (port_, speed_);
		
	ExternControl.Throttle = 0;	
	ExternControl.Yaw = 0;
	ExternControl.Pitch = 0;
	ExternControl.Roll = 0;
	ExternControl.Aux1 = 128;
    ExternControl.Aux2 = -128;
	ExternControl.Aux3 = 0;
	ExternControl.Aux4 = 0;
	
	while (ros::ok()){
		SendOutData(0x70, ExternControl, 0x08);
		/*ROS_INFO("Throttle: %3d    Yaw: %4d    Pitch: %4d    Roll: %4d",
						ExternControl.Throttle, ExternControl.Yaw, ExternControl.Pitch, ExternControl.Roll);*/
		ROS_INFO("Throttle: %3d",ExternControl.Throttle);
		ros::spinOnce();
		loop_rate.sleep();
	}
	CloseSerialInterface();
	return 0;
}
