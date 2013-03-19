#include <raposang_driver.h>

#include "ros/ros.h"

#include "raposang_msgs/RaposaPower.h"
#include "raposang_msgs/Empty.h"

using namespace std;

int fd;

// ------------------------------------------------
// Function: [Aux]
// ------------------------------------------------

bool RAPOSA_Stop_Using_Cable(raposang_msgs::Empty::Request &dummy, raposang_msgs::Empty::Response &dummyr) {

  ROS_INFO("[RAPOSANG-Driver] Service: Stop Using Cable."); 
   
	RAPOSA_Set_Reles(fd, 0x00);
	
	return true;
}


bool RAPOSA_Use_Cable(raposang_msgs::Empty::Request &dummy, raposang_msgs::Empty::Response &dummyr) {

  ROS_INFO("[RAPOSANG-Driver] Service: Use Cable.");
    
	RAPOSA_Set_Reles(fd, 0x87);
	
	return true;
}


// ------------------------------------------------
// Function: [Main]
// ------------------------------------------------

int main(int argc, char *argv[]) {

	int reads_per_second;
	string relays_interface;
	raposang_msgs::RaposaPower	raposapower;
	battery_status bat;

  ros::init(argc, argv, "raposang");
  ros::NodeHandle n;  
	ros::NodeHandle nh("~");
	
	nh.param("reads_per_second", reads_per_second, ODOMETRY_FREQ);	
	nh.param<std::string>("relays_port", relays_interface, "/dev/ttyUSBrelays");

	ros::ServiceServer serviceA = n.advertiseService("raposang/stop_using_cable", RAPOSA_Stop_Using_Cable);
	ros::ServiceServer serviceB = n.advertiseService("raposang/use_cable", RAPOSA_Use_Cable);
	
	ROS_INFO("[RAPOSANG-Driver] RAPOSA-NG Drivers for relays has started.");
	
	fd = RAPOSA_USB_Serial_Open(relays_interface.c_str(), BAUDRATE);

	if (fd!=-1 && RAPOSA_Check_Ports(fd, RELAYS)) {

		ros::Publisher pub  = n.advertise<raposang_msgs::RaposaPower>("raposang/power", 5);

		ros::Rate r(reads_per_second);

		while (ros::ok()) {

			bat = RAPOSA_Get_Battery_Status(fd);

			raposapower.header.stamp = ros::Time::now();
			
			raposapower.External_Source_on 								= bat.is_ES_On;
			raposapower.Relay_from_External_Source_on 		= bat.is_ES_Relay_On;
			raposapower.Relay_from_Electronics_Battery_on = bat.is_EB_Relay_On;
			raposapower.Relay_from_Motors_Battery_on      = bat.is_MB_Relay_On;
			raposapower.Relay_Status_Byte 								= bat.Relay_Status;
			raposapower.Motors_Battery_mV					= bat.MB_Voltage*100;
			raposapower.External_Source_mV				= bat.ES_Voltage*100;
			raposapower.Electronics_Battery_mV		= bat.EB_Voltage*100;

			pub.publish(raposapower);
			
    	ros::spinOnce();

   		r.sleep();
		}
		
	} else
		ROS_FATAL("[RAPOSANG-Driver] Missing RELAYS. Please check if USB is properly connected.");

	ROS_INFO("[RAPOSANG-Driver] Closing...");
	
	RAPOSA_USB_Serial_Close(fd);
	
	return -1;	
}

/* EOF */
