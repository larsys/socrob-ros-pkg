#include <raposang_driver.h>

#include "ros/ros.h"

#include "raposang_msgs/RaposaArm.h"
#include "raposang_msgs/RaposaVelocity.h"
#include "raposang_msgs/RaposaOdometry.h"
#include "raposang_msgs/GetArmAngle.h"
#include "raposang_msgs/Empty.h"

using namespace std;

int fd;
double click_to_dist_right, click_to_dist_left;
double angle_to_value_offset, angle_to_value, angle_prev, min_angle, max_angle, max_angle_dif;

struct floatlr odo_cum;
ros::Publisher  pub;

struct timeval prev;

bool prev_active;

// ------------------------------------------------
// Function: [Aux]
// ------------------------------------------------

void RAPOSA_Arm_Msg(const raposang_msgs::RaposaArm &raposaarm) {

  ROS_INFO("[RAPOSANG-Driver] Inclination command received (%d).", raposaarm.angle);
	RAPOSA_Set_Arm_Inclination(fd, (short) (raposaarm.angle*angle_to_value + angle_to_value_offset));

}

void RAPOSA_Motion_Msg(const raposang_msgs::RaposaVelocity &raposavelocity) {

  ROS_INFO("[RAPOSANG-Driver] Motion command received (%d,%d).",raposavelocity.left, raposavelocity.right);
	RAPOSA_Set_Velocity(fd, raposavelocity.left, raposavelocity.right);
	gettimeofday(&prev, NULL);
	prev_active = true;

}

bool RAPOSA_Get_Arm_Service(raposang_msgs::GetArmAngle::Request &dummy, raposang_msgs::GetArmAngle::Response &raposaodometryarm) {

  ROS_INFO("[RAPOSANG-Driver] Service: GetArmAngle.");  

	raposaodometryarm.angle = (short) angle_prev;
	
	return true;
}

bool RAPOSA_Stop_Arm_Service(raposang_msgs::GetArmAngle::Request &dummy, raposang_msgs::GetArmAngle::Response &raposaodometryarm) {

	short arm_inc;

  ROS_INFO("[RAPOSANG-Driver] Service: StopArmAngle.");  

	RAPOSA_Set_Arm_Inclination(fd, angle_prev * angle_to_value + angle_to_value_offset);

	raposaodometryarm.angle = (short) angle_prev;
	
	return true;
}

bool RAPOSA_Reset_Odometry(raposang_msgs::Empty::Request &dummy, raposang_msgs::Empty::Response &dummyr) {

  ROS_INFO("[RAPOSANG-Driver] Service: ResetOdometry.");  
	odo_cum.left  = 0.0;
	odo_cum.right = 0.0;
	
	return true;
}

void RAPOSA_Odometry_Timer(const ros::WallTimerEvent& event) {

	raposang_msgs::RaposaOdometry raposaodometry;

	raposaodometry.header.stamp = ros::Time::now();
	  
	shortlr clicks = RAPOSA_Get_Delta_XY_Clicks(fd);

	odo_cum.left  += ((float) clicks.left ) * click_to_dist_left;
	odo_cum.right += ((float) clicks.right) * click_to_dist_right;

	raposaodometry.track_left = odo_cum.left;
	raposaodometry.track_right = odo_cum.right;
						
	float angle = (RAPOSA_Get_Arm_Inclination(fd) - angle_to_value_offset) / angle_to_value;

	if ((angle > min_angle) && (angle < max_angle) && (abs(angle-angle_prev) < max_angle_dif)) {
		raposaodometry.arm_angle = angle;
		angle_prev = angle;
	} else 
		raposaodometry.arm_angle = angle_prev;
	
	pub.publish(raposaodometry);

}

// ------------------------------------------------
// Function: [Main]
// ------------------------------------------------

int main(int argc, char *argv[]) {

	int reads_per_second, odometry_frequency;	

	int time_before_stop;
	
	struct timeval curr;
	struct timeval dif;
	
	string motors_interface;

	bool reset_at_beginning;

	odo_cum.left  = 0.0;
	odo_cum.right = 0.0;

  ros::init(argc, argv, "raposang");
  ros::NodeHandle n;  
	ros::NodeHandle nh("~");
	
	nh.param("reads_per_second", reads_per_second, 20);
	nh.param("time_before_stop", time_before_stop, TIME_BEFORE_STOP);
	nh.param("odometry_frequency", odometry_frequency, ODOMETRY_FREQ);
	nh.param("click_to_dist_left", click_to_dist_left, CLICK_TO_DIST);
	nh.param("click_to_dist_right", click_to_dist_right, CLICK_TO_DIST);	

	nh.param("min_angle", min_angle, MIN_ANGLE);		
	nh.param("max_angle", max_angle, MAX_ANGLE);	
	nh.param("max_angle_dif", max_angle_dif, MAX_ANGLE_DIF);	
			
	nh.param("angle_to_value", angle_to_value, ANGLE_TO_VALUE);		
	nh.param("angle_to_value_offset", angle_to_value_offset, ANGLE_TO_VALUE_OFFSET);
		
	nh.param("reset_at_beginning", reset_at_beginning, true);
	
	nh.param<std::string>("motors_port", motors_interface, "/dev/ttyUSBmotors");

	gettimeofday(&prev, NULL);

	ROS_INFO("[RAPOSANG-Driver] RAPOSA-NG Drivers for motors has started.");

	fd = RAPOSA_USB_Serial_Open(motors_interface.c_str(), BAUDRATE);

	if (fd!=-1 && RAPOSA_Check_Ports(fd, MOTORS)) {
		
    ROS_INFO("[RAPOSANG-Driver] Ready for input commands.");

		/* Node setup */
		ros::Subscriber subM = n.subscribe("raposang/motion", 1, RAPOSA_Motion_Msg);
		ros::Subscriber subA = n.subscribe("raposang/arm", 1, RAPOSA_Arm_Msg);
		pub  = n.advertise<raposang_msgs::RaposaOdometry>("raposang/odometry", 1);
	
		ros::ServiceServer serviceStp = n.advertiseService("raposang/stop_arm", RAPOSA_Stop_Arm_Service);
		ros::ServiceServer serviceGet = n.advertiseService("raposang/get_arm", RAPOSA_Get_Arm_Service);
		ros::ServiceServer serviceOdo = n.advertiseService("raposang/reset_odometry", RAPOSA_Reset_Odometry);
		
		ros::Rate r(reads_per_second);
   
		ros::WallTimer timer = n.createWallTimer(ros::WallDuration(1.0/odometry_frequency), RAPOSA_Odometry_Timer); 

		angle_prev = 0.0;
		
		RAPOSA_Set_Arm_Inclination(fd, angle_to_value_offset);
		sleep(1.0);
		
		RAPOSA_Set_Velocity(fd, 0, 0);
		sleep(7.0);

		while (ros::ok()) {

			gettimeofday(&curr, NULL);
			timersub(&curr, &prev, &dif);
			if(dif.tv_sec >= time_before_stop) 
				RAPOSA_Set_Velocity(fd, 0, 0);
			
    	ros::spinOnce();

   		r.sleep();
		}
		
	} else
		ROS_FATAL("[RAPOSANG-Driver] Missing MOTORS. Please check if USB is properly connected.");

	ROS_INFO("[RAPOSANG-Driver] Closing...");
	
	RAPOSA_USB_Serial_Close(fd);
	
	return -1;	
}

/* EOF */
