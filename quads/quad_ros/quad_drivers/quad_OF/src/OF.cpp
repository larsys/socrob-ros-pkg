#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <math.h>
#include "mav_msgs/Height.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <math.h>

ros::Publisher OF_message;
double alpha, focal_length, current_height, delta_pitch, delta_roll;
//double scalar = 1.1*4, num_pixel = 30;
double sum_of_x =0,sum_of_y =0;
//int i=0;
double raw_dx=0,raw_dy=0;
//double roll, last_roll, pitch, last_pitch;
double last_roll, last_pitch;

double last_time, current_time;
ros::Duration t;
btScalar yaw, pitch, roll;

geometry_msgs::Vector3Stamped OF;


/////////////////////////////////////////////////////
//Calculate the Field of view for the specified lens
// focal_lenght should be in mm
/////////////////////////////////////////////////////

void calc_field_of_view(void){
  double d=30*60*pow(10,-6);
  alpha=2*atan2(d,2*focal_length*pow(10,-3));
}

void OF_RawCallback(geometry_msgs::Vector3 const &msg){
    sum_of_x=sum_of_x+msg.x*0.01;
    sum_of_y=sum_of_y+msg.y*0.01;
    raw_dx=msg.x;
    raw_dy=msg.y;
    //raw_dx=msg.x;
    //raw_dy=msg.y;

    /*if(i>10){
        sum_of_x=sum_of_x/10;
        sum_of_y=sum_of_y/10;
        OF.header.stamp=ros::Time::now();
        OF.vector.x=sum_of_x+(abs(sum_of_x)/alpha)*omega_y*0.1;
        OF.vector.y=sum_of_y+(abs(sum_of_y)/alpha)*omega_x*0.1;
        OF_message.publish(OF);
        i=0;
    }else{
        sum_of_x+=msg.x;
        sum_of_y+=msg.y;
        i+=1;
    }*/


    //OF.header.stamp=ros::Time::now();
    //OF.vector.x=((1/(num_pixel*scalar))*2*tan(alpha/2))*msg.x*current_height;
    //OF.vector.y=((1/(num_pixel*scalar))*2*tan(alpha/2))*msg.y*current_height;
    //OF.vector.x=msg.x-(msg.x/alpha)*omega_y*0.01;
    //OF.vector.y=msg.y-(msg.y/alpha)*omega_x*0.01;
    //ROS_WARN("Comp_X: %f Comp_Y: %f",(msg.x/alpha)*omega_y*0.01,(msg.y/alpha)*omega_x*0.01);
    //OF_message.publish(OF);
}

void imuCallback(sensor_msgs::Imu const &imu){

    //Get Euler from quaternions
    current_time=ros::Time::now().toSec();
    if(last_time>0){
        last_roll=roll;
        last_pitch=pitch;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imu.orientation, orientation);
        tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);

        delta_pitch=pitch-last_pitch;
        delta_roll=roll-last_roll;

        //Publish OF
        OF.header.stamp=ros::Time::now();
        //OF.vector.x=-(sum_of_x+0.082*((30/alpha)*delta_pitch));//-(0.18*(sin(pitch)-sin(last_pitch))*200/current_height));
        //OF.vector.y=-(sum_of_y+0.082*((30/alpha)*delta_roll));//-(0.18*(sin(roll)-sin(last_roll))*200/current_height));
        //double A=1.7622;//-0.3603;
        //double F=-2.9721;//-3.9923;
        //OF.vector.x=-(sum_of_x-((sin(pitch)-sin(pitch))/current_height)*F)+A*delta_pitch;
        //OF.vector.y=-(sum_of_y-((sin(roll)-sin(last_roll))/current_height)*F)+A*delta_roll;
        if(raw_dx==0){
            OF.vector.x=-sum_of_x;
        }else{
            OF.vector.x=-(sum_of_x+(30/alpha)*delta_pitch);
        }
        if(raw_dy==0){
            OF.vector.y=-sum_of_y;
        }else{
            OF.vector.y=-(sum_of_y+(30/alpha)*delta_roll);
        }
        OF_message.publish(OF);
        ROS_INFO("Comp Factor: %lf   %lf   %lf",OF.vector.y, sum_of_y, (30/alpha)*delta_roll);
    }
    else{
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imu.orientation, orientation);
        tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
    }
    last_time=current_time;
    sum_of_x=0;
    sum_of_y=0;

    /*omega_x=imu.angular_velocity.x;
    omega_y=imu.angular_velocity.y;
    OF.header.stamp=ros::Time::now();
    OF.vector.x=-(raw_dx+(abs(raw_dx)/alpha)*omega_y*0.1);
    OF.vector.y=-(raw_dy+(abs(raw_dy)/alpha)*omega_x*0.1);
    OF_message.publish(OF);*/
}

void heightCallback(const mav_msgs::Height& msg){
    //Update current height
    current_height=msg.height+0.1;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "quad_OF");
    ros::NodeHandle nh("~");

    nh.param("focal_length", focal_length, 12.0);
    calc_field_of_view();

    roll=0;
    last_roll=0;
    pitch=0;
    last_pitch=0;
    last_time = -1;
    current_time = -1;

    //Subscribing
    ros::Subscriber sub1 = nh.subscribe("/quad_height/height_to_base", 10, heightCallback);
    ros::Subscriber sub2 = nh.subscribe("/OF_Raw", 10, OF_RawCallback);
    ros::Subscriber sub3 = nh.subscribe("/imu/att", 10, &imuCallback);

    OF_message = nh.advertise<geometry_msgs::Vector3Stamped>("O_Flow", 10);

    ros::spin();
    return 0;
}
