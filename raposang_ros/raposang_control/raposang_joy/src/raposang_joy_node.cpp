// ------------------- INCLUDE -------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <math.h>

#include "ros/ros.h"
#include "raposang_msgs/RaposaVelocity.h"
#include "raposang_msgs/RaposaArm.h"
#include "raposang_msgs/RaposaOdometry.h"
#include "std_msgs/Float32.h"
#include "raposang_msgs/GetArmAngle.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"

// -------------------- DEFINE --------------------

#define FREQUENCY 100.0

#define minmax(a, min_arm, max_arm)		((a < max_arm) ? ((a > min_arm) ? a : min_arm) : max_arm)
#define check(a, s)										((a >= 0) && (a < s))


// ----------------- STRUCT ENUM ------------------

struct joypad {
	
	int default_vel_wheels;
	double default_vel_arm;
	double default_vel_pan;
	double default_vel_tilt;
	
	double turbo[4];
	
	int button_vel[4];
	int button_stop_arm;	
	int button_max_arm;	
	int button_min_arm;		
	int button_default_arm;
	int button_break;		
	int button_light;		
	int button_reset_pantilt;
	
	int axis_wheels;
	int axis_arm;
	int axis_pantilt;	
}; 

// ------------------- GLOBALS --------------------

ros::Publisher pubP, pubT, pubV, pubA, pubL;
ros::ServiceClient clientStop;

std_msgs::Empty dummy;

bool had_movement, asked_light, was_ok;

double dir, dirP, dirT;

double pan_precision;
double tilt_precision;
double arm_precision;

double max_arm, min_arm, reset_arm;
double reset_pan;
double reset_tilt;

struct joypad pad;

// ------------------- FUNCTION -------------------

bool RAPOSA_Check_Button(std::vector<int> v, int i) {
	return ((i>=0) && (i<v.size()) && (v[i]!=0));
}

bool RAPOSA_Check_Axis(std::vector<float> v, int i) {
	return ((i>=0) && (i<v.size()) && (v[i]!=0));
}

float RAPOSA_Refine_Controls(float a) {

	if (a>1.0f)
		return 1.0f;
	else if (a>0.5f)
		return ((2.0f/3.0f)*a+(1.0f/3.0f));
	else if (a>-0.5f)
		return (2.0f*a);
	else if (a>-1.0f)
		return ((2.0f/3.0f)*a-(1.0f/3.0f));
	else if (a<=-1.0f)
		return -1.0f;
}

void RAPOSA_Arm_Update(const raposang_msgs::RaposaOdometry &armmsg) {

	dir = armmsg.arm_angle;

}

void RAPOSA_Joy_Control(const sensor_msgs::Joy &joymsg) {

	std_msgs::Float32 						raposapantilt;
	raposang_msgs::RaposaVelocity raposavelocity;
	raposang_msgs::RaposaArm      raposaarm;

	float x, y;

	ROS_INFO("[RAPOSANG-JOY] Command received.");

	// Define Turbo factor

	double Turbo = (RAPOSA_Check_Button(joymsg.buttons,pad.button_vel[0]) ? joymsg.buttons[pad.button_vel[0]] * pad.turbo[0] : 0) +
								 (RAPOSA_Check_Button(joymsg.buttons,pad.button_vel[1]) ? joymsg.buttons[pad.button_vel[1]] * pad.turbo[1] : 0) +
								 (RAPOSA_Check_Button(joymsg.buttons,pad.button_vel[2]) ? joymsg.buttons[pad.button_vel[2]] * pad.turbo[2] : 0) +
								 (RAPOSA_Check_Button(joymsg.buttons,pad.button_vel[3]) ? joymsg.buttons[pad.button_vel[3]] * pad.turbo[3] : 0);
								 
	if (!Turbo)
		Turbo = 1.0;

	// Pan & Tilt movement
	
	if(RAPOSA_Check_Axis(joymsg.axes,2*pad.axis_pantilt)) {				
		dirP = minmax(dirP - joymsg.axes[2*pad.axis_pantilt]*pad.default_vel_pan*Turbo, 0.0, 180.0);
		if(raposapantilt.data != (int) dirP) {
			raposapantilt.data = dirP;
			pubP.publish(raposapantilt);	
		}	
	}

	if(RAPOSA_Check_Axis(joymsg.axes,2*pad.axis_pantilt+1)) {				
		dirT = minmax(dirT + joymsg.axes[2*pad.axis_pantilt+1]*pad.default_vel_tilt*Turbo, 0.0, 180.0);
		if(raposapantilt.data != (int) dirT) {				
			raposapantilt.data = dirT;		
			pubT.publish(raposapantilt);	
		}	
	}
	
	if (RAPOSA_Check_Button(joymsg.buttons, pad.button_reset_pantilt)) {
		dirP = dirT = raposapantilt.data = 90;		
		pubP.publish(raposapantilt);		
 		pubT.publish(raposapantilt);	
	}

	
	// Wheels movement	

	bool has_movement = RAPOSA_Check_Axis(joymsg.axes,2*pad.axis_wheels) || RAPOSA_Check_Axis(joymsg.axes,2*pad.axis_wheels+1) ;
	
	if ((!has_movement && had_movement) || RAPOSA_Check_Button(joymsg.buttons,pad.button_break)) {
		raposavelocity.left = raposavelocity.right = 0;
	  ROS_INFO("[RAPOSANG-JOY] Stopping RAPOSA (%d,%d).", raposavelocity.left, raposavelocity.right);
		pubV.publish(raposavelocity);
		had_movement = false;
	}
	else if(has_movement) {	

		x = joymsg.axes[2*pad.axis_wheels];
		y = joymsg.axes[2*pad.axis_wheels+1];

		raposavelocity.left  = (short) (RAPOSA_Refine_Controls(x+y) * pad.default_vel_wheels * Turbo);
		raposavelocity.right = (short) (RAPOSA_Refine_Controls(x-y) * pad.default_vel_wheels * Turbo);	
		
	  ROS_INFO("[RAPOSANG-JOY] Velocities sent to RAPOSA (%d,%d).", raposavelocity.left, raposavelocity.right);
		pubV.publish(raposavelocity);		
		had_movement = true;
	} 

	// On/Off Light

	if (RAPOSA_Check_Button(joymsg.buttons, pad.button_light))
		asked_light = true;
	else if(asked_light) {
		pubL.publish(dummy);
		asked_light = false;
	}	

	// Arm Control
	
	if (RAPOSA_Check_Button(joymsg.buttons, pad.button_default_arm)) {
		raposaarm.angle = dir = reset_arm;
  		ROS_INFO("[RAPOSANG-JOY] Reset. Inclination sent to RAPOSA (%f).", raposaarm.angle);
		pubA.publish(raposaarm);	
	} else if (RAPOSA_Check_Button(joymsg.buttons, pad.button_max_arm)) {
		raposaarm.angle = dir = max_arm;
  		ROS_INFO("[RAPOSANG-JOY] Increment. Inclination sent to RAPOSA (%f).", raposaarm.angle);
		pubA.publish(raposaarm);
	} else if (RAPOSA_Check_Button(joymsg.buttons, pad.button_min_arm)) {		
		raposaarm.angle = dir = min_arm;
  		ROS_INFO("[RAPOSANG-JOY] Decrement. Inclination sent to RAPOSA (%f).", raposaarm.angle);
		pubA.publish(raposaarm);
	} else if (RAPOSA_Check_Axis(joymsg.axes, 2*pad.axis_arm+1)) {
	  	raposaarm.angle = dir = minmax(dir + joymsg.axes[2*pad.axis_arm+1] * pad.default_vel_arm * Turbo, min_arm, max_arm);
		ROS_INFO("[RAPOSANG-JOY] Inclination sent to RAPOSA (%f).", dir);	
		pubA.publish(raposaarm);
	} 

	if (RAPOSA_Check_Button(joymsg.buttons, pad.button_stop_arm)) {
		raposang_msgs::GetArmAngle GetArmAngle_srv;
		clientStop.call(GetArmAngle_srv);
		raposaarm.angle = dir = GetArmAngle_srv.response.angle;			
		//had_arm_change = false;
	}
}

void RAPOSA_Define_Buttons() {
			
	if ((pad.button_vel[0]<=0)
	 || (pad.button_vel[1]<=0)
	 || (pad.button_vel[2]<=0)
	 || (pad.button_vel[3]<=0)
	 || (pad.button_light<=0)
	 || (pad.button_break<=0)	
	 || (pad.button_stop_arm<=0)
	 || (pad.button_max_arm<=0)	
	 || (pad.button_min_arm<=0)
	 || (pad.button_default_arm<=0)
	 || (pad.button_reset_pantilt<=0)) {
		 ROS_ERROR("[RAPOSANG-JOY] Button wrongly defined. Please identify each button with a positive ID.");
		 exit(1);
	 }

	if ((pad.axis_wheels<=0)
	 || (pad.axis_pantilt<=0)
	 || (pad.axis_arm<=0)) {
		 ROS_ERROR("[RAPOSANG-JOY] Axis wrongly defined. Please identify each button with a positive ID.");
		 exit(1);
	 }

	pad.button_vel[0]--;
	pad.button_vel[1]--;
	pad.button_vel[2]--;		
	pad.button_vel[3]--;
	pad.button_light--;
	pad.button_break--;	
	pad.button_stop_arm--;
	pad.button_max_arm--;	
	pad.button_min_arm--;	
	pad.button_default_arm--;
	pad.button_reset_pantilt--;			
	
	pad.axis_wheels--;
	pad.axis_arm--;
	pad.axis_pantilt--;		
	
}

// ------------------------------------------------
// Function: [Main]
// ------------------------------------------------

int main(int argc, char *argv[]) {

	std_msgs::Float32 						raposapantilt;
	raposang_msgs::RaposaArm      raposaarm;

	double frequency;

	const double normal_freq = FREQUENCY;

	raposang_msgs::GetArmAngle GetArmAngle_srv;

	ROS_INFO("[RAPOSANG-JOY] Raposa-joy has started.");
  ros::init(argc, argv, "raposang_joy");
  
  ros::NodeHandle n;
	ros::NodeHandle nh("~");	
	
	ros::Subscriber subJ = n.subscribe("input_joy", 1, RAPOSA_Joy_Control);
	ros::Subscriber subA = n.subscribe("input_odometry", 1, RAPOSA_Arm_Update);

	pubV = n.advertise<raposang_msgs::RaposaVelocity>("output_motion", 10);
	pubA = n.advertise<raposang_msgs::RaposaArm>("output_arm",1);
	pubP = n.advertise<std_msgs::Float32>("output_pan",1);
	pubT = n.advertise<std_msgs::Float32>("output_tilt",1);	
	pubL = n.advertise<std_msgs::Empty>("output_light",1);

	ros::ServiceClient clientGet  = n.serviceClient<raposang_msgs::GetArmAngle>("/raposang/get_arm");			
	clientStop = n.serviceClient<raposang_msgs::GetArmAngle>("/raposang/stop_arm");
	
	// Read parameters
	
	nh.param("max_arm",	 	 max_arm, 	  30.0);
	nh.param("min_arm",	 	 min_arm, 	 -20.0);	
		
	nh.param("reset_arm",	 	 reset_arm, 	 0.0);		
	nh.param("reset_pan",	 	 reset_pan, 	 90.0);	
	nh.param("reset_tilt",	 reset_tilt, 	 90.0);	
		
	nh.param("button_default_arm",	 pad.button_default_arm, 	 -1);			
	nh.param("button_stop_arm", 		 pad.button_stop_arm, 	 	 -1);	
	nh.param("button_max_arm",			 pad.button_max_arm, 			 -1);
	nh.param("button_min_arm",			 pad.button_min_arm,			 -1);	
	nh.param("button_vel_2",				 pad.button_vel[1],				 -1);
	nh.param("button_vel_3", 				 pad.button_vel[2],				 -1);		
	nh.param("button_vel_1",				 pad.button_vel[0],				 -1);
	nh.param("button_vel_4", 				 pad.button_vel[3], 			 -1);	
	nh.param("button_reset_pantilt", pad.button_reset_pantilt, -1);		
	nh.param("button_light",				 pad.button_light, 			 	 -1);	
	nh.param("button_break",				 pad.button_break, 			 	 -1);	
			
	nh.param("default_vel_wheels",			pad.default_vel_wheels, 500);	
	nh.param("default_vel_arm",					pad.default_vel_arm,	  1.0);		
	nh.param("default_vel_pan",					pad.default_vel_pan,	  1.0);	
	nh.param("default_vel_tilt",				pad.default_vel_tilt, 	1.0);	

	nh.param("frequency", frequency, FREQUENCY);	
				
	nh.param("turbo_1", pad.turbo[0], 0.2);	
	nh.param("turbo_2", pad.turbo[1], 0.5);
	nh.param("turbo_3", pad.turbo[2], 1.5);	
	nh.param("turbo_4", pad.turbo[3], 2.0);		

	nh.param("axis_wheels",			 pad.axis_wheels,			 -1);	
	nh.param("axis_arm",				 pad.axis_arm,				 -1);
	nh.param("axis_pantilt", 		 pad.axis_pantilt,		 -1);	

	RAPOSA_Define_Buttons();

	// Normalize speed for inclination arm and pan & tilt

	pad.default_vel_arm  *=  normal_freq/frequency;
	pad.default_vel_pan  *=  normal_freq/frequency;
	pad.default_vel_tilt *=  normal_freq/frequency;
	
	had_movement = asked_light = was_ok = false;
	
	clientGet.call(GetArmAngle_srv);
	
	dir = raposaarm.angle = GetArmAngle_srv.response.angle;	
	dirP = dirT = raposapantilt.data = 90;
	pubP.publish(raposapantilt);		
 	pubT.publish(raposapantilt);	
 	
	// ROS loop

	ros::spin();

	/*ros::Rate r(FRE);
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}*/

	return 0;
}

/* EOF */
