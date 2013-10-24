#include "ros/ros.h"

enum {P_GAIN,I_GAIN,D_GAIN,I_MAX,I_MIN};

typedef struct{
    double desired;
    double z_ref;
    double zd_ref;
    double zdd_ref;
    double Ref_Rate;
} Reference;

typedef struct{
    ros::Time time_last;
    double last;
    ros::Time time_current;
    double current;
} Height_str;

double PID_height(double desired_height, Height_str Height, double height_gain[5], double t, double Mass, double roll, double pitch, ros::Publisher AccZ);
void ZKalman_Init(void);
double ZKalman_newZMeasurement(double z, double zdd);
