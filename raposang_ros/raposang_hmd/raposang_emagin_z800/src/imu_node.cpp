#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <libz800.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <raposang_msgs/ImuRaw.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[]) {
		
	int frequency;
	float gyr[3], acc[3], mag[3];
	double qf[4];

	std::string frame_id, child_frame_id;

	tf::Transform transform;	
	sensor_msgs::Imu sensor_imu;
	raposang_msgs::ImuRaw imuraw;

	ros::init(argc, argv, "raposang_emagin_z800");
  ros::NodeHandle n;  
  ros::NodeHandle nh("~");
	
	tf::TransformBroadcaster br;
		
  ros::Publisher pubSensorImu = n.advertise<sensor_msgs::Imu>("imu_emagin_z800", 1);	
  ros::Publisher pubImuRaw = n.advertise<raposang_msgs::ImuRaw>("wrap920", 1);	
    	
	nh.param("frequency",	frequency, 30);
	nh.param<std::string>("frame_id", frame_id, "/world");
	nh.param<std::string>("child_frame_id", child_frame_id, "/z800");	
			
	ROS_INFO("[RAPOSANG-EMAGIN-Z800] Connecting to Emagin z800...");	
	z800_device *dev = z800_connect(7);
	if(dev == NULL)	{
		ROS_FATAL("[RAPOSANG-EMAGIN-Z800] Connection to Emagin z800 failed.");
		exit(1);
	}

	z800_set_debug_level(dev, 5);

	ROS_INFO("[RAPOSANG-EMAGIN-Z800] Connected!");	

	int firmware = z800_get_firmware_version(dev);
	if(firmware < 0)
		ROS_ERROR("[RAPOSANG-EMAGIN-Z800] Failed to acquire firmware version.");
	else
		ROS_INFO("[RAPOSANG-EMAGIN-Z800] Firmware version: %d.%d.", firmware >> 8, firmware & 0xff);

	ROS_INFO("[RAPOSANG-EMAGIN-Z800] Getting Raw data from Emagin z800.");

  //z800_set_stereo_mode(dev, Z800_TRUE );	

	ros::Rate rate(frequency);			
	while(ros::ok()){		

		z800_poll_tracker(dev);
		z800_get_calibrated_tracker_data(dev, &gyr, &acc, &mag);
		z800_get_orientation(dev, &qf);
		
		sensor_imu.header.frame_id = frame_id;
    sensor_imu.header.stamp = ros::Time::now();		
    
		sensor_imu.orientation.x = qf[0];
		sensor_imu.orientation.y = qf[1];
		sensor_imu.orientation.z = qf[2];		
		sensor_imu.orientation.w = qf[3];		
		sensor_imu.angular_velocity.x = gyr[0];
		sensor_imu.angular_velocity.y = gyr[1];
		sensor_imu.angular_velocity.z = gyr[2];	
		sensor_imu.linear_acceleration.x = acc[0];
		sensor_imu.linear_acceleration.y = acc[1];
		sensor_imu.linear_acceleration.z = acc[2];	

		sensor_imu.orientation_covariance[0] = 
		sensor_imu.angular_velocity_covariance[0] = 
		sensor_imu.linear_acceleration_covariance[0] = -1;

		imuraw.header.frame_id = frame_id;
    imuraw.header.stamp = sensor_imu.header.stamp;		

		imuraw.magnetometer[0] = mag[0];
		imuraw.magnetometer[1] = mag[1];
		imuraw.magnetometer[2] = mag[2];		
		imuraw.gyroscope[0] = gyr[0];
		imuraw.gyroscope[1] = gyr[1];
		imuraw.gyroscope[2] = gyr[2];	
		imuraw.accelerometer[0] = acc[0];
		imuraw.accelerometer[1] = acc[1];
		imuraw.accelerometer[2] = acc[2];	


		transform.setOrigin(tf::Vector3(0, 0, 0));																		
		transform.setRotation(tf::Quaternion(qf[0], qf[1], qf[2], qf[3]));
		br.sendTransform(tf::StampedTransform(transform, 
																					sensor_imu.header.stamp, 
																					frame_id, child_frame_id));

		//ROS_INFO("[RAPOSANG-EMAGIN-Z800] qx=%3.3f qy=%3.3f qz=%3.3f qw=%3.3f.", qf[0], qf[1], qf[2], qf[3]);

    pubSensorImu.publish(sensor_imu); 
    pubImuRaw.publish(imuraw);    
    
    rate.sleep();

	}
}
