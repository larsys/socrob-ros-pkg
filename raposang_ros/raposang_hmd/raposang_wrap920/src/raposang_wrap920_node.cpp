#include "wrap920_tracker.hpp"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "raposang_msgs/ImuRPY.h"
#include "raposang_msgs/srvBool.h"
#include "raposang_msgs/Empty.h"

#include "std_msgs/Float32.h"

#include "ros/ros.h"
#include "raposang_msgs/ImuRaw.h"

#define PI_TO_DEG 180.0/3.1415

#define PROB1 0.5
#define PROB0 (1.0-PROB1)

bool control_PT, first;

bool RAPOSA_Control_PT(raposang_msgs::srvBool::Request &control, raposang_msgs::srvBool::Response &dummyr) {

	if (control.data) {
		ROS_INFO("[RAPOSANG-EMAGIN-Z800] Service: Control PT with HMD Activated."); 
		control_PT = true;
    first = true;
	} else {
		ROS_INFO("[RAPOSANG-EMAGIN-Z800] Service: Control PT with HMD Deactivated."); 
		control_PT = false;
	}

	return true;
}

#define BUF_GYRO 500

//------------------------------------------
//                  MAIN
//------------------------------------------
 
int main(int argc, char **argv) {
  
  wrap920_tracker wr;
  
  raposang_msgs::ImuRaw imu;
  raposang_msgs::ImuRPY imurpy;
     
  sensor_msgs::Imu imu_ros;    
  geometry_msgs::Vector3Stamped imu_mag;
  std_msgs::Float32 yaw, pitch, roll;
    
  wr.connect();  
  
  double prev, now, dif, yaw_gyr = 0;
  float yaw_off = 0, pitch_off = 0, roll_off = 0;
  float yaw_d = 0, pitch_d = 0, roll_d = 0;
  float pitch_old = 0, roll_old = 0, yaw_old = 0;
  float pitch_thres_old = 0, roll_thres_old = 0, yaw_thres_old = 0;
        
  shortimu raw;
  floatimu cal;  
    
  ros::init(argc, argv, "raposang_wrap920");
  ros::NodeHandle n;
	
  ros::Publisher pubImu = n.advertise<raposang_msgs::ImuRaw>("wrap920", 5);		
  ros::Publisher pubImuSensor = n.advertise<sensor_msgs::Imu>("wrap920_sensor", 5);		
  ros::Publisher pubImuMag = n.advertise<geometry_msgs::Vector3Stamped>("wrap920_mag", 5);	    
  ros::Publisher pubP = n.advertise<std_msgs::Float32>("output_pan",1); 
  ros::Publisher pubT = n.advertise<std_msgs::Float32>("output_tilt",1);	 
  ros::Publisher pubRPY = n.advertise<raposang_msgs::ImuRPY>("wrap920_rpy",1);	  
  
	ros::ServiceServer serviceStop = n.advertiseService("control_PT_with_HMD", RAPOSA_Control_PT);

  control_PT = true;
  first = true;
  
  prev = now = ros::Time::now().toSec();
  
  wr.compensateGyroOffset(true);
   
  while (ros::ok()) {
    
    ros::spinOnce();
    
    now = ros::Time::now().toSec();
    
    if (wr.update()) {
      
      dif = now-prev;
      if ( dif > 1.0/20.0) {
        
        prev = now;
        
        raw = wr.getRawData();
        cal = wr.getCalibratedData();
        
        imu.header.stamp = imu_ros.header.stamp = imu_mag.header.stamp = ros::Time::now();
        
        imu.accelerometer[0] = raw.accelerometer.x;
        imu.accelerometer[1] = raw.accelerometer.y;
        imu.accelerometer[2] = raw.accelerometer.z;
        
        imu.magnetometer[0] = raw.magnetometer.x;
        imu.magnetometer[1] = raw.magnetometer.y;
        imu.magnetometer[2] = raw.magnetometer.z;    
            
        imu.gyroscope[0] = raw.gyroscope_low.x;
        imu.gyroscope[1] = raw.gyroscope_low.y;
        imu.gyroscope[2] = raw.gyroscope_low.z;

        imu_mag.vector.x = cal.magnetometer.x;
        imu_mag.vector.y = cal.magnetometer.y;
        imu_mag.vector.z = cal.magnetometer.z;   
        
        imu_ros.angular_velocity.x = cal.gyroscope_low.x;
        imu_ros.angular_velocity.y = cal.gyroscope_low.y;
        imu_ros.angular_velocity.z = cal.gyroscope_low.z;       

        imu_ros.linear_acceleration.x = cal.accelerometer.x;
        imu_ros.linear_acceleration.y = cal.accelerometer.y;
        imu_ros.linear_acceleration.z = cal.accelerometer.z;   
        
        pubImu.publish(imu);
        pubImuSensor.publish(imu_ros);       
        pubImuMag.publish(imu_mag);
        
        pitch_d = wr.getPitch();

        roll_d = wr.getRoll();

        //yaw_d = atan2(cal.magnetometer.x, cal.magnetometer.y) * PI_TO_DEG;
        yaw_gyr -= dif*cal.gyroscope_low.x;
        
        
        if (first) {
          //yaw_off = yaw_d;
          yaw_off = yaw_gyr = 0.0;
          pitch_off = pitch_d;
          roll_off = roll_d;
          yaw_old = yaw_thres_old =yaw_d;          
          pitch_old = yaw_thres_old =pitch_d;
          roll_old = yaw_thres_old =roll_d;
          yaw.data = 90.0;
          pitch.data = 90.0;  
          roll.data = 90.0;                  
          first = false;
        }
        
        pitch_d = PROB1*pitch_d + PROB0*pitch_old;
        roll_d  = PROB1*roll_d  + PROB0*roll_old;
        yaw_d   = PROB1*yaw_gyr + PROB0*yaw_old;
        
        //yaw_d   = PROB1*yaw_d + PROB0*yaw_old;
        
        if (abs(pitch_d-pitch_thres_old) > 0.5){
          pitch.data = pitch_d - pitch_off + 90;
          pitch_thres_old = pitch_d;
        }
        roll.data = roll_d - roll_off + 90;
        
        if (abs(yaw_d-yaw_thres_old) > 0.5) {
          yaw.data = yaw_d - yaw_off + 90;   
          yaw_thres_old = yaw_d;
        }
        
       // ROS_INFO("Pitch: %3.3f Yaw: %3.3f, Roll: %3.3f", pitch.data, yaw.data, roll.data);
               
        if (control_PT) {
          imurpy.roll = roll_d;
          imurpy.pitch = pitch_d;
          imurpy.yaw = yaw_d;
          pubP.publish(yaw);
          pubT.publish(pitch);
          pubRPY.publish(imurpy);
        }	
        
        pitch_old = pitch_d;         
        roll_old = roll_d;        
        yaw_old = yaw_gyr;
                      
        
      }
    }
    
   // rate.sleep();  
  }
  
  return 0;
}

