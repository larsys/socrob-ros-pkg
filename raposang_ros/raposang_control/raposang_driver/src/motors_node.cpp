#include <raposang_driver.h>

#include "ros/ros.h"

#include "raposang_msgs/RaposaArm.h"
#include "raposang_msgs/RaposaVelocity.h"
#include "raposang_msgs/RaposaOdometry.h"
#include "raposang_msgs/GetArmAngle.h"
#include "raposang_msgs/Empty.h"

#define SENSIBILITY 0.5

using namespace std;

int fd;
double click_to_dist_right, click_to_dist_left;
double angle_to_value_offset, angle_to_value, angle_prev, min_angle, max_angle, max_angle_dif;

struct floatlr odo_cum, odo_cum_raw;
ros::Publisher  pub, pubraw;

double prev;

double direction, angle_to_compensate;

bool prev_active, publish_raw_odometry;

// ------------------------------------------------
// Function: [Aux]
// ------------------------------------------------

void RAPOSA_Arm_Msg(const raposang_msgs::RaposaArm &raposaarm) {

    ROS_INFO("[RAPOSANG-Driver] Inclination command received (%f).", raposaarm.angle);
    RAPOSA_Set_Arm_Inclination(fd, (short) round(raposaarm.angle*angle_to_value + angle_to_value_offset));

}

void RAPOSA_Motion_Msg(const raposang_msgs::RaposaVelocity &raposavelocity) {

	ROS_INFO("[RAPOSANG-Driver] Motion command received (%d,%d).",raposavelocity.left, raposavelocity.right);
	RAPOSA_Set_Velocity(fd, raposavelocity.left, raposavelocity.right);
	prev = ros::WallTime::now().toSec();
	prev_active = true;

}

bool RAPOSA_Get_Arm_Service(raposang_msgs::GetArmAngle::Request &dummy, raposang_msgs::GetArmAngle::Response &raposaodometryarm) {

  ROS_INFO("[RAPOSANG-Driver] Service: GetArmAngle.");  

	raposaodometryarm.angle = round(angle_prev);
	
	return true;
}

bool RAPOSA_Stop_Arm_Service(raposang_msgs::GetArmAngle::Request &dummy, raposang_msgs::GetArmAngle::Response &raposaodometryarm) {

	ROS_INFO("[RAPOSANG-Driver] Service: StopArmAngle (%f).", angle_prev);  

	RAPOSA_Set_Arm_Inclination(fd, (short) round((angle_prev+direction) * angle_to_value + angle_to_value_offset));

	raposaodometryarm.angle = angle_prev+direction;
	
	return true;
}

bool RAPOSA_Reset_Odometry(raposang_msgs::Empty::Request &dummy, raposang_msgs::Empty::Response &dummyr) {

    ROS_INFO("[RAPOSANG-Driver] Service: ResetOdometry.");  
	odo_cum.left = odo_cum.right = 0.0;
	if(publish_raw_odometry) {
		odo_cum_raw.left = odo_cum_raw.right = 0.0;
    }
	return true;
}

void RAPOSA_Odometry_Timer(const ros::WallTimerEvent& event) {

	raposang_msgs::RaposaOdometry raposaodometry;

	raposaodometry.header.stamp = ros::Time::now();

	shortlr clicks = RAPOSA_Get_Delta_XY_Clicks(fd);
	short angle_raw = RAPOSA_Get_Arm_Inclination(fd);

	if(publish_raw_odometry) {
		odo_cum_raw.left  += (double) clicks.left ;
		odo_cum_raw.right += (double) clicks.right;
		raposaodometry.track_left = odo_cum_raw.left;
		raposaodometry.track_right = odo_cum_raw.right;
		raposaodometry.arm_angle = (double) angle_raw;
		pubraw.publish(raposaodometry);
	}

	odo_cum.left  += ((double) clicks.left ) * click_to_dist_left;
	odo_cum.right += ((double) clicks.right) * click_to_dist_right;

	raposaodometry.track_left = odo_cum.left;
	raposaodometry.track_right = odo_cum.right;

	double angle = (((double) angle_raw) - angle_to_value_offset) / angle_to_value;
	double angle_dif = angle-angle_prev;


	if ((angle > min_angle) && (angle < max_angle) && (abs(angle_dif) < max_angle_dif)) {
		ROS_INFO("[RAPOSANG-Driver] Odometry: Good angle reading (%f).", angle);
		if ((angle_dif > -SENSIBILITY) && (angle_dif < SENSIBILITY))
			direction = 0;
		else if (angle_dif > SENSIBILITY)
			direction = angle_to_compensate;
		else if (angle_dif < -SENSIBILITY)
			direction = -angle_to_compensate;
		raposaodometry.arm_angle = angle;
		angle_prev = angle;
	} else {
		ROS_INFO("[RAPOSANG-Driver] Odometry: Bad angle reading (%f).", angle);
		raposaodometry.arm_angle = angle_prev;
	}

	pub.publish(raposaodometry);



}

// ------------------------------------------------
// Function: [Main]
// ------------------------------------------------

int main(int argc, char *argv[]) {

	int reads_per_second, odometry_frequency;	

	int time_before_stop;
	
	string motors_interface;

	bool reset_at_beginning;

	direction = 0;
	
	odo_cum.left  = 0.0;
	odo_cum.right = 0.0;

	odo_cum_raw.left  = 0.0;
	odo_cum_raw.right = 0.0;

    ros::init(argc, argv, "raposang");
    ros::NodeHandle n;  
	ros::NodeHandle nh("~");
	
	nh.param("publish_raw_odometry", publish_raw_odometry, true);

	nh.param("reads_per_second", reads_per_second, 20);
	nh.param("time_before_stop", time_before_stop, TIME_BEFORE_STOP);
	nh.param("odometry_frequency", odometry_frequency, ODOMETRY_FREQ);
	nh.param("click_to_dist_left", click_to_dist_left, CLICK_TO_DIST);
	nh.param("click_to_dist_right", click_to_dist_right, CLICK_TO_DIST);	

	nh.param("min_angle", min_angle, MIN_ANGLE);		
	nh.param("max_angle", max_angle, MAX_ANGLE);	
	nh.param("max_angle_dif", max_angle_dif, MAX_ANGLE_DIF);	

	nh.param("angle_to_compensate", angle_to_compensate, 2.0);
			
	nh.param("angle_to_value", angle_to_value, ANGLE_TO_VALUE);		
	nh.param("angle_to_value_offset", angle_to_value_offset, ANGLE_TO_VALUE_OFFSET);
		
	nh.param("reset_at_beginning", reset_at_beginning, true);
	
	nh.param<std::string>("motors_port", motors_interface, "/dev/ttyUSBmotors");

	ROS_INFO("[RAPOSANG-Driver] RAPOSA-NG Drivers for motors has started.");

	fd = RAPOSA_USB_Serial_Open(motors_interface.c_str(), BAUDRATE);

	if (fd!=-1 && RAPOSA_Check_Ports(fd, MOTORS)) {
		
   		ROS_INFO("[RAPOSANG-Driver] Ready for input commands.");

		/* Node setup */
		ros::Subscriber subM = n.subscribe("raposang/motion", 1, RAPOSA_Motion_Msg);
		ros::Subscriber subA = n.subscribe("raposang/arm", 1, RAPOSA_Arm_Msg);

		pub  = n.advertise<raposang_msgs::RaposaOdometry>("raposang/odometry", 1);

		if (publish_raw_odometry)
			pubraw = n.advertise<raposang_msgs::RaposaOdometry>("raposang/odometryraw", 1);
	
		ros::ServiceServer serviceStp = n.advertiseService("raposang/stop_arm", RAPOSA_Stop_Arm_Service);
		ros::ServiceServer serviceGet = n.advertiseService("raposang/get_arm", RAPOSA_Get_Arm_Service);
		ros::ServiceServer serviceOdo = n.advertiseService("raposang/reset_odometry", RAPOSA_Reset_Odometry);
		
		ros::Rate r(reads_per_second);
   
		ros::WallTimer timer = n.createWallTimer(ros::WallDuration(1.0/odometry_frequency), RAPOSA_Odometry_Timer); 

		angle_prev = 0.0;
		
		RAPOSA_Set_Arm_Inclination(fd, (short) angle_to_value_offset);
		RAPOSA_Set_Velocity(fd, 0, 0);

		prev_active = false;

		while (ros::ok()) {

			if(prev_active && ((ros::WallTime::now().toSec() - prev) >= time_before_stop)) {
				RAPOSA_Set_Velocity(fd, 0, 0);	
				prev_active = false;
			}		
				
				
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
