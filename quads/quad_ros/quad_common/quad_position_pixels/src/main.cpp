#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "quad_can_driver/Attitude.h"

#define PI 3.14159256

// limit 10 degrees
#define angle_MAX 0.1745
#define angle_MIN -0.1745


quad_can_driver::Attitude att_msg;
ros::Publisher att_pub;
double K;
std::string _pixels_topic_subscribe;

typedef struct{
    double roll;
    double pitch;
}direction_vector;

direction_vector roll_pitch;


void Callback_pixels_disp(const geometry_msgs::PointStamped pixel_disp){

    double x_des = pixel_disp.point.x;
    double y_des = pixel_disp.point.y;




    roll_pitch.roll = -K * x_des;
    roll_pitch.pitch = K * y_des;

}


int main(int argc, char **argv){

    roll_pitch.roll = roll_pitch.pitch = 0.0;

    ros::init(argc, argv, "quad_position_pixels");
    ros::NodeHandle nh("~");

    ros::Rate rate(50);

    nh.param<double>("K", K, 0.01);
    nh.param<std::string>("pixels_topic_subscribe", _pixels_topic_subscribe, "/pixel_disp");

    ros::Subscriber sub = nh.subscribe(_pixels_topic_subscribe,5,Callback_pixels_disp);

    //Publishing
    att_pub = nh.advertise<quad_can_driver::Attitude>("/AttitudeCtrl", 5);

    while(ros::ok()){

        if(roll_pitch.roll > angle_MAX)
            roll_pitch.roll=angle_MAX;

        if(roll_pitch.roll < angle_MIN)
            roll_pitch.roll=angle_MIN;

        if(roll_pitch.pitch > angle_MAX)
            roll_pitch.pitch=angle_MAX;

        if(roll_pitch.pitch < angle_MIN)
            roll_pitch.pitch=angle_MIN;



        // reduce the value incrementally
        roll_pitch.roll *= 0.8;
        roll_pitch.pitch *= 0.8;


        att_msg.roll=roll_pitch.roll;
        att_msg.pitch=roll_pitch.pitch;

        att_pub.publish( att_msg );
        ROS_INFO("roll: %lf , pitch: %lf",att_msg.roll,att_msg.pitch);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;

}
