#include "quad_height/pid_height.h"
#include <math.h>

double p_term, d_term, i_term;
double i_error = 0;
double ThrustMax = 23.2472;
double ThrustMin = 6.5241;
double ThrustCoefA = 0.102613151321317;
double ThrustCoefB = 3.352893193381934;


double smooth(double last, double current, int Gain, double t)
{	
	double cmd;
	int Smooth_Gain = Gain;
	cmd = t*((Smooth_Gain*current)+(last/t)-(Smooth_Gain*last));
	return cmd;
}

int ThrustConversion(double AccZ, double Mass){
	
		int thrust = 0;
		ROS_INFO("AccZ: %f",AccZ);
		if (AccZ<=ThrustMin){
			thrust = 20;
		}else if (AccZ>=ThrustMax){
			thrust = 210;
		}else{
			thrust = round((AccZ*Mass-ThrustCoefB)/ThrustCoefA);
			ROS_INFO("Thrust: %d",thrust);
		}
		return thrust;
}

double PID_height(double desired_height, double current_height, double last_height, double height_gain[5], double t, double Mass, double roll, double pitch){

	double p, d, i, i_max, i_min;
	double error;
	double d_error;
	double G = 9.80665;
	
	current_height=smooth(last_height,current_height,1,t);
	ROS_INFO("Height: %f %f",last_height,current_height);
	
	error = desired_height - current_height;
	
	p = height_gain[P_GAIN];
	d = height_gain[D_GAIN];
	i = height_gain[I_GAIN];
	i_max = height_gain[I_MAX];
	i_min = height_gain[I_MIN];
		
	// Calculate proportional contribution to command
	p_term = p * error;

	// Calculate the integral error
	i_error = i_error + t * error;
	
	//Calculate integral contribution to command
	i_term = i * i_error;

	// Limit i_term so that the limit is meaningful in the output
	if (i_term > i_max){
		i_term = i_max;
		i_error=i_term/i;
	}
	else if (i_term < -i_min){
		i_term = -i_min;
		i_error=i_term/i;
	}
	ROS_INFO("Integral Error: %f %f",i_error, i_term);

	// Calculate the derivative error
	d_error = (current_height - last_height) / t;

	// Calculate derivative contribution to command
	d_term = d * d_error;
	
	//double AccZ =((p_term + i_term - d_term)+G)/(cos(pitch)*cos(roll));
	double AccZ =((p_term + i_term - d_term)+G);
	
	int cmd = ThrustConversion (AccZ, Mass);
	//ROS_INFO("Thrust Command: %d",cmd);	  
	return cmd;
}
