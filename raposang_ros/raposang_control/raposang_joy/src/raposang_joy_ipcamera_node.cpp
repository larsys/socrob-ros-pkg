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
#include "raposang_msgs/PanasonicCameraControl.h"
#include "sensor_msgs/Joy.h"

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

ros::Publisher pubC;
ros::ServiceClient clientStop;

float x, y;

struct joypad pad;

// ------------------- FUNCTION -------------------

bool RAPOSA_Check_Button(std::vector<int> v, int i) {
	return ((i>=0) && (i<v.size()) && (v[i]!=0));
}

bool RAPOSA_Check_Axis(std::vector<float> v, int i) {
	return ((i>=0) && (i<v.size()) && (v[i]!=0));
}

void RAPOSA_Joy_Control(const sensor_msgs::Joy &joymsg) {

	raposang_msgs::PanasonicCameraControl camera_control;
	
	if (joymsg.buttons[7])
		camera_control.turbo = true;
		
	if (joymsg.buttons[8])
		camera_control.homePosition = true;
	else if (joymsg.buttons[2])
		camera_control.zoomTele = true;		
	else if (joymsg.buttons[3])
		camera_control.zoomWide = true;		
	else if (joymsg.buttons[1])	
		camera_control.mouse = true;		
	else if (joymsg.buttons[0])	{
		x = 640/2;
		y = 480/2;	
	}
	else if (joymsg.axes[4] > 0)
		camera_control.panLeft = true;
	else if (joymsg.axes[4] < 0)
		camera_control.panRight = true;
	else if (joymsg.axes[5] > 0)
		camera_control.tiltUp = true;
	else if (joymsg.axes[5] < 0)
		camera_control.tiltDown = true;			
			
	if (joymsg.axes[0] != 0.0 || joymsg.axes[1] != 0.0) {
		x -= 20*joymsg.axes[0];
		y -= 20*joymsg.axes[1];
		
		x = (x < 0.0) ? 0.0 : ((x > 640) ? 640 : x);
		y = (y < 0.0) ? 0.0 : ((y > 480) ? 480 : y);

	}
				
	camera_control.x = x;
	camera_control.y = y;	
	
	pubC.publish(camera_control);	


}

// ------------------------------------------------
// Function: [Main]
// ------------------------------------------------

int main(int argc, char *argv[]) {

	const double normal_freq = FREQUENCY;

	ROS_INFO("[RAPOSANG-JOY] Raposa-joy-ipcamera has started.");
  ros::init(argc, argv, "raposang_joy_ipcamera");
  
  ros::NodeHandle n;
	ros::NodeHandle nh("~");	
	
	ros::Subscriber subJ = n.subscribe("input_joy", 1, RAPOSA_Joy_Control);

	pubC = n.advertise<raposang_msgs::PanasonicCameraControl>("output_ipcamera", 1);

	x = 640/2;
	y = 480/2;

	// ROS loop

	ros::spin();

	return 0;
}

/* EOF */
