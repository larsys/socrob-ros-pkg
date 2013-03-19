#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <tf/tf.h>
#include <Ivy/timer.h>

#define DEG2RAD 3.14159265359/180

ros::Publisher imu_message;
sensor_msgs::Imu imu_data;
tf::Quaternion q;

/* 
 * Parse the IMU string filling the output using the IMU format message
 * Parameters:  frame_id;
 * 				linear_acceleration_stdev;
 * 				orientation_stdev;
 * 				angular_velocity_stdev;
 */
 
void ATTCallback (IvyClientPtr app, void* data , int argc, char **argv)
{
	double att_unit_coef= 0.0139882;
	double phi, theta, yaw;
		
	sscanf(argv[0],"%lf %lf %lf %*d %*d %*d",&phi,&theta,&yaw);
	phi*=-att_unit_coef*DEG2RAD;
	theta*=att_unit_coef*DEG2RAD;
	yaw*=att_unit_coef*DEG2RAD;
	
	q.setRPY(phi,theta,yaw);

	imu_data.header.stamp = ros::Time::now();
	imu_data.orientation.x=q.x();
	imu_data.orientation.y=q.y();
	imu_data.orientation.z=q.z();
	imu_data.orientation.w=q.w();
	
	//ROS_INFO("Phi %f; Theta %f; Yaw %f", phi,theta,yaw);
	//ROS_INFO("q1 %f; q2 %f; q3 %f; q4 %f", q.x(),q.y(),q.z(),q.w());
		
	imu_message.publish(imu_data);
}

void GYROCallback (IvyClientPtr app, void* data , int argc, char **argv)
{
	double gyro_unit_coef= 0.0139882;
	
	sscanf(argv[0],"%lf %lf %lf",&imu_data.angular_velocity.x,&imu_data.angular_velocity.y,&imu_data.angular_velocity.z);
	imu_data.angular_velocity.x*=-gyro_unit_coef*DEG2RAD;
	imu_data.angular_velocity.y*=gyro_unit_coef*DEG2RAD;
	imu_data.angular_velocity.z*=gyro_unit_coef*DEG2RAD;
	
	imu_data.header.stamp = ros::Time::now();
	imu_message.publish(imu_data);
}

void ACCELCallback (IvyClientPtr app, void* data , int argc, char **argv)
{
	double acc_unit_coef= 0.0009766;
	
	sscanf(argv[0],"%lf %lf %lf",&imu_data.linear_acceleration.x,&imu_data.linear_acceleration.y,&imu_data.linear_acceleration.z);
	imu_data.linear_acceleration.x*=-acc_unit_coef;
	imu_data.linear_acceleration.y*=acc_unit_coef;
	imu_data.linear_acceleration.z*=acc_unit_coef;
	
	imu_data.header.stamp = ros::Time::now();
	imu_message.publish(imu_data);
}

void ROSCallback (TimerId id, void *user_data, unsigned long delta) 
{ 
	if (!ros::ok()){
		IvyStop();
		exit(0);
	}
}

int main(int argc, char **argv)
{
	double angular_velocity_covariance, angular_velocity_stdev;
	double linear_acceleration_covariance, linear_acceleration_stdev;
	double orientation_covariance, orientation_stdev;
	std::string frame_id;
	
	// Initializing ROS
	ros::init(argc, argv, "imu");
	ros::NodeHandle nh("~");
	
	imu_message = nh.advertise<sensor_msgs::Imu>("data", 1000);
	
	// Getting Parameters
	nh.param<std::string>("frame_id", frame_id, "imu");
    imu_data.header.frame_id = frame_id;

    nh.param("linear_acceleration_stdev", linear_acceleration_stdev, 0.0); 
    nh.param("orientation_stdev", orientation_stdev, 0.0); 
    nh.param("angular_velocity_stdev", angular_velocity_stdev, 0.0); 

    angular_velocity_covariance = pow(angular_velocity_stdev,2);
    orientation_covariance = pow(orientation_stdev,2);
    linear_acceleration_covariance = pow(linear_acceleration_stdev,2);
    
    // Fill IMU Message with covariances
    imu_data.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imu_data.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imu_data.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    imu_data.angular_velocity_covariance[0] = angular_velocity_covariance;
    imu_data.angular_velocity_covariance[4] = angular_velocity_covariance;
    imu_data.angular_velocity_covariance[8] = angular_velocity_covariance;
    
    imu_data.orientation_covariance[0] = orientation_covariance;
    imu_data.orientation_covariance[4] = orientation_covariance;
    imu_data.orientation_covariance[8] = orientation_covariance;

	// Initializing Ivy
	IvyInit ("imu_parser", "'Imu Parser' is READY!", 0, 0, 0, 0);
	IvyStart("127.255.255.255");
	TimerRepeatAfter (TIMER_LOOP, 500, ROSCallback, 0);
	
	// Binding Messages
	IvyBindMsg(ATTCallback, 0, "BOOZ2_AHRS_EULER(.*)");
	IvyBindMsg(GYROCallback, 0, "IMU_GYRO_SCALED(.*)");
	IvyBindMsg(ACCELCallback, 0, "IMU_ACCEL_SCALED(.*)");
	
	ROS_INFO("IMU: Starting Aquisition Loop");

	IvyMainLoop();
	
	return 0;
}
