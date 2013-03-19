// ------------------------------------------------
// Includes
// ------------------------------------------------

#include "ros/ros.h"

#include "raposang_msgs/RaposaPanTilt.h"
#include "raposang_msgs/Empty.h"
#include "std_msgs/Float32.h"

// ------------------------------------------------
// Defines
// ------------------------------------------------

#define READS_PER_SECOND 10
#define PAN_OFFSET				0.0
#define TILT_OFFSET				0.0

// ------------------------------------------------
// Globals
// ------------------------------------------------

double pan_offset, tilt_offset;
ros::Publisher pub_pan, pub_tilt;

// ------------------------------------------------
// Function: [Aux]
// ------------------------------------------------

void RAPOSA_Pan_Tilt(const raposang_msgs::RaposaPanTilt &raposapantilt) {

  ROS_INFO("[RAPOSANG-Driver] Pan&Tilt command received (%f,%f).", raposapantilt.pan, raposapantilt.tilt);
	
	std_msgs::Float32 pan, tilt;
	
	pan.data = raposapantilt.pan - pan_offset;
	tilt.data = raposapantilt.tilt - tilt_offset;
	
	pub_pan.publish(pan);
	pub_tilt.publish(tilt);
}

// ------------------------------------------------
// Function: [Main]
// ------------------------------------------------

int main(int argc, char *argv[]) {

	int reads_per_second;

  ros::init(argc, argv, "raposang");
  
  ros::NodeHandle n;  
	ros::NodeHandle nh("~");

	nh.param("reads_per_second",  reads_per_second,  READS_PER_SECOND);
	nh.param("pan_offset",  pan_offset,  PAN_OFFSET);
	nh.param("tilt_offset", tilt_offset, TILT_OFFSET);	

	ROS_INFO("[RAPOSANG-Driver] RAPOSA-NG Drivers for Pan&Tilt motor has started.");

	ros::Subscriber sub = n.subscribe("raposang/pantilt", 1, RAPOSA_Pan_Tilt);
	pub_pan  = n.advertise<std_msgs::Float32>("arduino_pan",  5);
	pub_tilt = n.advertise<std_msgs::Float32>("arduino_tilt", 5);
	
	ros::Rate r(reads_per_second);

	std_msgs::Float32 pan, tilt;
	
	pan.data = 90;
	tilt.data = 90;	
	
	pub_pan.publish(pan);
	pub_tilt.publish(tilt);

	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("[RAPOSANG-Driver] Closing...");
	
	return -1;	
}

/* EOF */
