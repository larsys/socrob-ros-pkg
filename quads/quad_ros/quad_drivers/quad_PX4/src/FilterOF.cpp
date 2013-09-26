#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <math.h>
#include "tf/transform_datatypes.h"
#include <Eigen/Eigen>
#include <Eigen/LU>
#include "px_comm/OpticalFlow.h"
#include "std_msgs/Float64.h"
#include "quad_PX4/KState.h"


btScalar yaw, pitch, roll;
bool K_init = false, Az_init = false;

ros::Publisher Kstate_pub;

typedef struct{
    ros::Time time_last;
    double last;
    ros::Time time_current;
    double current;
} Az_str;

Az_str Az;

//Kalman Variables
Eigen::Matrix<double, 6, 6> A, P;
Eigen::Matrix<double, 6, 2> B, K;
Eigen::Matrix<double, 6, 1> Xhat;
Eigen::Matrix<double, 2, 1> U, Resid, OF;
Eigen::Matrix<double, 4, 4> Q;
Eigen::Matrix<double, 6, 4> W;
Eigen::Matrix<double, 2, 6> C;
Eigen::Matrix<double, 2, 2> R;
Eigen::Matrix<double, 3, 3> Rroll, Rpitch, Ryaw, RotMat;
Eigen::Matrix<double, 3, 1> VecG, VecDir, VecAz;

void XYKalman_Init(){
    Xhat << 0, 0, 0, 0, 0, 0;
    P << 0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0,
         0,0,0,0,0,0;
    A << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    B << 0,0,
         0,0,
         0,0,
         0,0,
         0,0,
         0,0;
    W << 0,0,0,0,
         0,0,0,0,
         0,0,0,0,
         0,0,0,0,
         0,0,1,0,
         0,0,0,1;
    Q << 0.01,  0,      0,      0,
         0,     0.01,   0,      0,
         0,     0,      0.01,   0,
         0,     0,      0,      0.01;
    R << 0.1,   0,
         0,     0.1;
    VecG << 0,0,9.81;
    VecDir << 0,0,1;
    VecAz << 0,0,0;
    C << 0,0,1,0,0,0,
         0,0,0,1,0,0;
    K_init=true;
}

double XYKalman_Predict(double AccZ, double _dt){
    if(K_init){
        A(0,2)=_dt;
        A(1,3)=_dt;
        A(2,4)=_dt;
        A(3,5)=_dt;
        A(0,4)=pow(_dt,2)/2;
        A(1,5)=pow(_dt,2)/2;
        B(0,0)=pow(_dt,2)/2;
        B(1,1)=pow(_dt,2)/2;
        B(2,0)=_dt;
        B(3,1)=_dt;
        W(0,0)=pow(_dt,2)/2;
        W(1,1)=pow(_dt,2)/2;
        W(2,0)=_dt;
        W(3,1)=_dt;

        Rroll << 1, 0,          0,
                 0, cos(roll), -sin(roll),
                 0, sin(roll), cos(roll);
        Rpitch << cos(pitch), 0, sin(pitch),
                  0,          1, 0,
                  -sin(pitch),0, cos(pitch);
        Ryaw << cos(yaw), -sin(yaw), 0,
                sin(yaw), cos(yaw), 0,
                0,        0,        1;
        RotMat=Rroll*Rpitch*Ryaw;
        VecAz = AccZ*RotMat*VecDir-VecG;
        U(0,0)=VecAz(0,0);
        U(1,0)=VecAz(1,0);

        // Predict
        // Propagate the state estimate and covariance matrix
        Xhat=A*Xhat+B*U;
        P=A*P*A.transpose()+W*Q*W.transpose();
        return 1;
    }
    return 0;
}

double XYKalman_Update(Eigen::Matrix<double, 2, 1> Z){
    if(K_init){
        //Update
        // Calculate the Kalman gain
        K = (P * C.transpose()) * (C * P * C.transpose() + R).inverse();
        // Calculate the measurement residual
        Resid = Z - C*Xhat;
        // Update the state and error covariance estimate
        Xhat = Xhat + K*Resid;
        P = (Eigen::MatrixXd::Identity(6, 6)-K*C)*P;
        return 1;
    }
    return 0;
}

void AccZCallback(std_msgs::Float64 const &msg){
    if(!Az_init){
        Az.time_last=ros::Time::now();
        Az.last=msg.data;
        Az.time_current=Az.time_last;
        Az.current=Az.last;
        Az_init = true;
    } else{
        Az.time_last=Az.time_current;
        Az.last=Az.current;
        Az.time_current=ros::Time::now();
        Az.current=msg.data;
        ros::Duration t = Az.time_current - Az.time_last;
        XYKalman_Predict(Az.current,t.toSec());
    }
}

void imuCallback(sensor_msgs::Imu const &imu){
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imu.orientation, orientation);
    tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
}

void PX4Callback(px_comm::OpticalFlow const &msg){
    OF(0,0)=msg.velocity_x;
    OF(1,0)=msg.velocity_y;
    XYKalman_Update(OF);

    quad_PX4::KState Kstate;
    Kstate.header.stamp=ros::Time::now();
    Kstate.x=Xhat(0,0);
    Kstate.y=Xhat(1,0);
    Kstate.vx=Xhat(2,0);
    Kstate.vy=Xhat(3,0);
    Kstate.fx=Xhat(4,0);
    Kstate.fy=Xhat(5,0);

    Kstate_pub.publish(Kstate);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "FilterOF");
    ros::NodeHandle n;

    //Subscribing
    ros::Subscriber sub1 = n.subscribe("/px4flow/opt_flow", 1, PX4Callback);
    ros::Subscriber sub2 = n.subscribe("/imu/data", 10, &imuCallback);
    ros::Subscriber sub3 = n.subscribe("Applied_AccZ", 10, &AccZCallback);

    //Publishing
    Kstate_pub = n.advertise<quad_PX4::KState>("HState", 10);

    XYKalman_Init();
    ros::spin();
    /*ros::Rate loop_rate(20);

    while (ros::ok()){

        quad_PX4::KState Kstate;
        Kstate.header.stamp=ros::Time::now();
        Kstate.x=Xhat(0,0);
        Kstate.y=Xhat(1,0);
        Kstate.vx=Xhat(2,0);
        Kstate.vy=Xhat(3,0);
        Kstate.fx=Xhat(4,0);
        Kstate.fy=Xhat(5,0);

        Kstate_pub.publish(Kstate);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;*/
}
