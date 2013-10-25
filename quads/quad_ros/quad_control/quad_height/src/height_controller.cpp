#include "ros/ros.h"
#include "quad_can_driver/Thrust.h"
#include "mav_msgs/Height.h"
#include "quad_height/pid_height.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "px_comm/OpticalFlow.h"

// Variables for Reference Generator
Reference Ref;

Height_str Height;
tfScalar yaw, pitch, roll;

double RC;
ros::Publisher zhat_pub, Accz_pub, thrust_pub, desired_pub;

bool init_control = false, init_ref = false;

void desired_heightCallback(std_msgs::Float32 const &msg){
    Ref.desired=msg.data;
    init_ref = true;
}

void imuCallback(sensor_msgs::Imu const &imu){
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(imu.orientation, orientation);
  	tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
}

void heightCallback(const mav_msgs::Height& msg){
    //Update last height
    Height.time_last=Height.time_current;
    Height.last=Height.current;
    //Update current height
    Height.time_current=msg.header.stamp;
    Height.current=msg.height;
    init_control=true;
    //New Measure -> Run the Estimator
    double delta = ZKalman_newZMeasurement(Height.current, 0);
    std_msgs::Float64 zhat;
    zhat.data=delta;
    zhat_pub.publish(zhat);
}

// Second Order Filter for Reference Generator
void Ref_Generator(double t){
    double zd_ref_old=Ref.zd_ref;
    double z_ref_old=Ref.z_ref;
    Ref.zdd_ref=((Ref.desired-Ref.z_ref)*Ref.Ref_Rate)-Ref.zd_ref;
    Ref.zd_ref=zd_ref_old+Ref.zdd_ref*t;
    Ref.z_ref=z_ref_old+Ref.zd_ref*t;
}

/*
* Returns the sign of a double
* 1        if the value is positive
* 0        if the value is 0
* -1       if the value is negative
*/
/*int sign(double v){
       return v>0?1:(v<0?-1:0);
}*/
/*double Ramp(double ref_u, double uref_old){
	
	if(fabs(ref_u-uref_old)> Ref_Rate)
		ref_u = uref_old + sign(ref_u-uref_old)*Ref_Rate;
		
	return ref_u;
}*/

int main(int argc, char **argv){
	
	ros::init(argc, argv, "quad_height");
    ros::NodeHandle n, nl("~");

    // Variables
    bool pub_reference;
    int cmd = 0;
    double height_gain[5];
    double Mass;

    //Set Reference Variables
    Ref.desired=0;
    Ref.zdd_ref=0;
    Ref.zd_ref=0;
    Ref.z_ref=0;
    Ref.Ref_Rate=0.3;

    // Height Structure
    Height.current=0;
    Height.last=0;
    Height.time_current=ros::Time::now();
    Height.time_last=Height.time_current;

    std_msgs::Float32 zref_pub;
    quad_can_driver::Thrust thrust_msg;

    //Read Parameters
    nl.param("height_p", height_gain[P_GAIN], 1.0);
    nl.param("height_i", height_gain[I_GAIN], 0.01);
    nl.param("height_d", height_gain[D_GAIN], 1.3);
    nl.param("height_imax", height_gain[I_MAX], 1.0);
    nl.param("height_imin", height_gain[I_MIN], 1.0);
    nl.param("mass",Mass,1.60);
    nl.param("pub_ref",pub_reference,true);

    //Subscribing
	ros::Subscriber sub1 = n.subscribe("/quad_height/height_to_base", 10, heightCallback);
	ros::Subscriber sub2 = n.subscribe("/imu/data", 10, &imuCallback);
    ros::Subscriber sub4 = n.subscribe("/desired_height", 10, &desired_heightCallback);
	
    //Publishing
	thrust_pub = n.advertise<quad_can_driver::Thrust>("ThrustCtrl", 10);
    if(pub_reference) desired_pub = n.advertise<std_msgs::Float32>("Z_Ref", 10);
    zhat_pub = n.advertise<std_msgs::Float64>("xhat", 10);
    Accz_pub = n.advertise<std_msgs::Float64>("Applied_AccZ", 10);

    ros::Rate loop_rate(10);
	ros::Time last_time, current_time;
	ros::Duration t;

	last_time = ros::Time::now();
	current_time = ros::Time::now();
    ZKalman_Init();

	while (ros::ok()){
		
		last_time = current_time;
		current_time = ros::Time::now();
		t = current_time - last_time;

        //Reference Generator
        Ref_Generator(t.toSec());
        ROS_WARN("Z_Ref: %f",Ref.z_ref);

        //Publish Reference
        if(pub_reference){
            zref_pub.data= Ref.z_ref;
            desired_pub.publish(zref_pub);
        }

        //PID Controller
        if(init_control && init_ref){
            cmd = PID_height(Ref.z_ref, Height, height_gain, t.toSec(), Mass, roll, pitch, Accz_pub);
        }
			
        if(Ref.z_ref<=0.1 && Height.current<=0.1) cmd=0;
        ROS_INFO("Thrust Command: %d",cmd);

		// Send message to quad_can_interface
		thrust_msg.header.stamp = ros::Time::now();
		thrust_msg.thrust = cmd;
		thrust_pub.publish(thrust_msg);

        //rosspin and rate
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
