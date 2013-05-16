#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <float.h>
#include <math.h>

// C++
#include <iostream>0.017453293
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>

// IMU
#include "sensor_msgs/Imu.h"

// Odometry
#include "raposang_msgs/RaposaOdometry.h"

// Pose
#include <nav_msgs/Odometry.h>

// ------------------------------------------------------------
//                          DEFINES
// ------------------------------------------------------------

#define DEGTORAD 0.017453293
#define WHEEL_DIST 0.42

// ------------------------------------------------------------
//                          STRUCT
// ------------------------------------------------------------

struct wheel_vel{
	
	double left, right, arm;

	wheel_vel(double _left = 0.0, double _right = 0.0, double _arm = 0.0) {		
		left  = _left;
		right = _right;
		arm   = _arm;		
	};	
	
	Eigen::Vector3d getTranslation() {
		
		const double ang = (right - left)/WHEEL_DIST;
		const double T   = (left + right)/2.0;
		
		Eigen::Vector3d t( T*cos(ang), T*sin(ang), 0.0);
		
		return t;
		
	};

	Eigen::Matrix3d	getRotationArm() {
		
		Eigen::Matrix3d R;
		
		double arm_rad = arm * DEGTORAD;
		
		R << cos(arm_rad), 0,  sin(arm_rad),
				            0, 1,             0,
				-sin(arm_rad), 0,  cos(arm_rad);
		
		return R;
	}
	
};

struct angular_vel{
	
	double x, y, z;

	angular_vel(double _x = 0.0, double _y = 0.0, double _z = 0.0) {		
		x = _x;
		y = _y;
		z = _z;		
	};
	
	Eigen::Quaterniond getQuaternionForAngularChange(double time) {

		const double wn  = sqrt(x*x + y*y + z*z);
		const double ang = wn * time / 2;
		const double swn = sin(ang);
		const double cwn = cos(ang);	
		const double wxn = (wn != 0.0) ? x/wn : 0.0;
		const double wyn = (wn != 0.0) ? y/wn : 0.0;
		const double wzn = (wn != 0.0) ? z/wn : 0.0;
		
		Eigen::Quaterniond qR(cwn, swn*wxn, swn*wyn, swn*wzn);
		
		return qR;
		
	};
	
	
};

// ------------------------------------------------------------
//                         	 CLASS
// ------------------------------------------------------------

class Odometry{
	
	ros::NodeHandle n;	
		
	ros::Subscriber sub_imu;
	ros::Subscriber sub_odo;
	ros::Publisher  pub_data;
	
	Eigen::Quaterniond q;
	
	nav_msgs::Odometry odometry;
	raposang_msgs::RaposaOdometry odo_prev;
	
	angular_vel w_imu;
	wheel_vel   v_wheel;
	
	bool first_odo;
	
	public:
	
		// Constructor
	
		Odometry() {
			
			q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
			
			first_odo = true;
			
			sub_odo = n.subscribe("input_odo", 1, &Odometry::getOdo, this);		
			sub_imu = n.subscribe("input_imu", 1, &Odometry::getImu, this);	
			
			pub_data = n.advertise<nav_msgs::Odometry>("output_pose", 1);			
				
		};
		
		// Destructor
		
		~Odometry() {		
		};

		// You spin me round baby
		
		void rosSpin() {					
			
			double dt;
			
			Eigen::Vector3d t(0.0, 0.0, 0.0);
			Eigen::Vector3d t_new(0.0, 0.0, 0.0);
			Eigen::Vector3d t_vel(0.0, 0.0, 0.0);
			Eigen::Vector3d t_imu(0.0, 0.0, 0.0);
			Eigen::Vector3d t_arm(-0.19, 0.0, 0.02);
			Eigen::Quaterniond q_imu;
			
			ros::Rate rate(15);	
			
			ros::Time time_prev, time_now;
			
			time_prev = ros::Time::now();

			while(ros::ok()) {	
				
				ros::spinOnce();
				
				time_now = ros::Time::now();			
				dt = (time_now - time_prev).toSec();
				time_prev = time_now;		
				
				// Compute Rotation
				
				q_imu = w_imu.getQuaternionForAngularChange(dt);
								
				q = q * q_imu;
								
				// Compute Translation		
				
				t_imu = v_wheel.getRotationArm()*v_wheel.getTranslation() - q_imu.toRotationMatrix()*t_arm + t_arm;
				
				t_new = t + q.toRotationMatrix()*t_imu;
				
				t_vel = (t_new - t)/dt;
				
				t = t_new;
								
				setPose(t, q, t_new);		
				
				pub_data.publish(odometry);
						
				rate.sleep();
			}			
			
		};
		
	private:
	
		void getImu(const sensor_msgs::Imu &imu){
			
			w_imu.x = -imu.angular_velocity.x;
			w_imu.y =  imu.angular_velocity.y;
			w_imu.z = -imu.angular_velocity.z;		

			//ROS_INFO("[IMU] Received IMU data! x=%f y=%f z=%f", w_imu.x, w_imu.y, w_imu.z);
					
		}; 
		
		void getOdo(const raposang_msgs::RaposaOdometry &odo){ 
			
			if(first_odo) 
				first_odo = false;
			else {
				v_wheel.right = odo.track_right - odo_prev.track_right;
				v_wheel.left  = odo.track_left  - odo_prev.track_left;				
			}
			
			v_wheel.arm = odo.arm_angle;
				
			odo_prev.track_right = odo.track_right;
			odo_prev.track_left  = odo.track_left;	
					
		};
		
		void setPose(Eigen::Vector3d t, Eigen::Quaterniond q, Eigen::Vector3d t_vel) {

			odometry.header.stamp    = ros::Time::now();
			odometry.header.frame_id = "/nav";
			odometry.child_frame_id = "/base_link";
					
			odometry.pose.pose.position.x = t(0);
			odometry.pose.pose.position.y = t(1);		
			odometry.pose.pose.position.z = t(2);	
			
			odometry.pose.pose.orientation.w = q.w();
			odometry.pose.pose.orientation.x = q.x();		
			odometry.pose.pose.orientation.y = q.y();	
			odometry.pose.pose.orientation.z = q.z();

			odometry.twist.twist.linear.x = t_vel(0);
			odometry.twist.twist.linear.y = t_vel(1);		
			odometry.twist.twist.linear.z = t_vel(2);	
			
			odometry.twist.twist.angular.x = w_imu.x;
			odometry.twist.twist.angular.y = w_imu.y;		
			odometry.twist.twist.angular.z = w_imu.z;
			
		};
		
	};

