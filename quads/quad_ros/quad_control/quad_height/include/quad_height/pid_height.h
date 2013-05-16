#include "ros/ros.h"

enum {P_GAIN,I_GAIN,D_GAIN,I_MAX,I_MIN};

double PID_height(double desired_height, double current_height, double last_height, double height_gain[5], double t, double Mass, double roll, double pitch);

