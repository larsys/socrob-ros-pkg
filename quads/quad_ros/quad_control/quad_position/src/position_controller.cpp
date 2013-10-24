#include "ros/ros.h"
#include "quad_can_driver/Attitude.h"
#include <geometry_msgs/Vector3Stamped.h>
#include "px_comm/OpticalFlow.h"

#define PI 3.14159256

double K;
quad_can_driver::Attitude att_msg;
ros::Publisher att_pub, OF_pub;

//Low Pass Filter Variables
bool initialized = false;

typedef struct{
    ros::Time time_last;
    double last_x;
    double last_y;
    ros::Time time_current;
    double current_x;
    double current_y;
} OF_str;

OF_str OF;

double Ts =0.01;
double tau = 0.2;
double alpha = Ts/tau;
px_comm::OpticalFlow OF_Fil;



/*void OFCallback(geometry_msgs::Vector3Stamped const &msg){

    double theta_d = -K*msg.vector.x;
    double phi_d = K*msg.vector.y;

    ROS_INFO("Phi_d: %f   Theta_d: %f", phi_d, theta_d);

    if(abs(theta_d)>0.068){
        theta_d=0.068*(theta_d/abs(theta_d));
    }

    if(abs(phi_d)>0.068){
        phi_d=0.068*(phi_d/abs(phi_d));
    }

    att_msg.pitch=theta_d;
    att_msg.roll=phi_d;

    att_pub.publish(att_msg);
}*/

void PX4Callback(px_comm::OpticalFlow const &msg){

    if(!initialized){
        OF.last_x = msg.velocity_x;
        OF.last_y = msg.velocity_y;
        OF.current_x = OF.last_x;
        OF.current_y = OF.last_y;
        initialized = true;
    } else{

        OF.last_x = OF.current_x;
        OF.last_y = OF.current_y;

        //Low Pass Filter
        OF.current_x = OF.last_x+alpha*(msg.velocity_x-OF.last_x);
        OF.current_y = OF.last_y+alpha*(msg.velocity_y-OF.last_y);
        OF_Fil.header.stamp=ros::Time::now();
        OF_Fil.velocity_x = OF.current_x;
        OF_Fil.velocity_y = OF.current_y;
        OF_pub.publish(OF_Fil);

        double theta_d = K*OF.current_x;
        double phi_d = K*OF.current_y;

        /*double theta_d = K*msg.velocity_x;
        double phi_d = K*msg.velocity_y;*/

        /*if(abs(theta_d)>0.068){
            theta_d=0.068*(theta_d/abs(theta_d));
        }

        if(abs(phi_d)>0.068){
            phi_d=0.068*(phi_d/abs(phi_d));
        }*/

        ROS_INFO("Phi_d: %f   Theta_d: %f", phi_d, theta_d);
        att_msg.pitch=theta_d;
        att_msg.roll=phi_d;

        att_pub.publish(att_msg);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "quad_position");
    ros::NodeHandle n, nh("~");

    //Read Parameters
    nh.param("P_gain", K, 0.16);

    //Subscribing
    //ros::Subscriber sub1 = n.subscribe("/quad_OF/O_Flow", 10, OFCallback);
    ros::Subscriber sub2 = n.subscribe("/px4flow/opt_flow", 1, PX4Callback);

    //Publishing
    att_pub = n.advertise<quad_can_driver::Attitude>("AttitudeCtrl", 1);
    OF_pub = n.advertise<px_comm::OpticalFlow>("/opt_flow_filt", 1);

    ros::spin();

    return 0;
}
