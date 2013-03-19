// ------------------------------------------------------------
//                         INCLUDE
// ------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <float.h>
#include <math.h>

// C++
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

// IMU
#include "sensor_msgs/Imu.h"

// Image
#include <cv.h>
#include <cxcore.h>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include <highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/PointCloud.h>
#include <raposang_msgs/RaposaSlam.h>
#include <image_transport/image_transport.h>
#include <raposang_msgs/RaposaFeatures.h>

// Odometry
#include "raposang_msgs/RaposaOdometry.h"

// TF
#include <tf/transform_broadcaster.h>

// 

//EOF

// ------------------------------------------------------------
//                         DEFINES
// ------------------------------------------------------------

// Options

#define _X_ 0
#define _Y_ 1
#define _Z_ 2

#define FRAMEID_MAP						"/raposang_map"
#define FRAMEID_ROBOT					"/raposang_pose"

#define USE_FLANN // Comment this line if you don't want to use normal matching.
#define USE_IMU 							true
#define USE_ODO								true
#define USE_FEAT							false

#define WITH_STEREO						  true
#define WITH_MONO			 					 true
#define NO_OUTSIDERS					false

#define RANDOM_FEATS					false

#define SYNC_PRED_WITH_UPD		true
#define ARM_ANGLE_ALWAYS_ZERO	false
#define CALIBRATE_IMU 				true
#define FEATURE_REMOVAL				true

#define ALMOST_ZERO						2.2204e-16

#define READS_PER_SECOND			100

#define WINDOW_LEFT 					"Window Left"
#define WHEEL_DIAMETER				0.18
#define WHEEL_DISTANCE				0.42

#define SCALE_STEREO_STR			1.0

// Features

#define MAX_NEW_LANDMARKS			20
#define MIN_FEATURES_PER_FRAME		10
#define MIN_DIST							20.0

// Standard Deviations - Predict

#define INIT_STD_LIN_VEL			1.0			    // m/s
#define INIT_STD_ANG_VEL			1.0			    // rad/s

#define STD_LIN_VEL						1.0			    // m/s
#define STD_ANG_VEL						1.0			    // rad/s
#define STD_IMU_VEL						1.0			    // rad/s
#define STD_WHEEL_VEL					1.0			    // m/s
#define STD_ARM								1.0			    // rad/s

// New Feature

#define INIT_INV_DEPTH				0.5				// 1/m
#define STD_PIXEL_IMG				  1.0			    
#define INIT_STD_INV_DEPTH					0.5			  

// Removal Filtering

#define REMOVAL_GAIN					0.5
#define REMOVAL_THRESHOLD			0.1
#define REMOVAL_START					1.0  				// Should always be 1.

// SURF Parameters

#define FEATURE_DETECTOR			"surf"

#define SURF_HESSIAN_THRESHOLD    	100.0
#define SURF_EXTENDED_DESCRIPTORS 	false
#define SURF_NR_OCTAVES							3
#define SURF_NR_OCTAVE_LAYERS				4

// Intrinsic Camera Parameters

#define IMAGE_SIZE_X		640
#define IMAGE_SIZE_Y		480
#define IMAGE_SCALE			0.5

#define IMAGE_ORIGIN_X  319.3656005859375
#define IMAGE_ORIGIN_Y  254.4078369140625

#define IMAGE_FOCAL_X		285.06625366210938
#define IMAGE_FOCAL_Y		285.06625366210938

// ------------------------------------------------------------
//                        NAMESPACES
// ------------------------------------------------------------

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace cv_bridge;

// ------------------------------------------------------------
//                         TYPEDEF
// ------------------------------------------------------------

enum feat_det { fd_surf, fd_sift, fd_orb };
enum cam_name { cam_left = 1, cam_right = -1, cam_stereo = 0};

typedef Matrix<double, 13, 13> Matrix13d;
typedef Matrix<double,  6,  6> Matrix6d;
typedef Matrix<double,  6,  1> Vector6d;
typedef Matrix<double,  7,  1> Vector7d;

typedef struct _imu_w{
	
	double x;
	double y;
	double z;
} imu_w;

typedef struct _odometry{
	
	double left;
	double right;
	double arm;
} odometry;

typedef struct _feat_filter{

	int total;
	int matched;
	VectorXd id_real;	
	VectorXd id_type;	
	
	std::vector<cv::KeyPoint> k;	
	cv::Mat d;

	
} Feat_filter;

typedef struct _Map_info{

	int total;

	std::vector<cv::KeyPoint> k;	
	cv::Mat d;
	
} Map_info;

typedef struct _State{

	MatrixXd P;	
	VectorXd x;
	Vector7d x_odo;	
	Map_info l;
	VectorXd f;
		
} State;

typedef struct _Stuff{

	std::string frameid_map;
	std::string frameid_robot;	

	int max_features;
	int min_features_per_frame;

	int reads_per_second;

	double scale_stereo_str;

	bool sync_predict_with_update;
	bool use_imu;
	bool use_odo;
	bool use_feat;
	
	bool with_stereo;
	bool with_mono;	
	bool no_outsiders;
	bool random_feats;
	
	bool do_update;
	bool do_predict;	
	bool calibrate_imu;
	bool arm_angle_always_zero;
	bool feature_removal;
	bool use_gyro_only;
	
	feat_det feature_detector;
	
	int robot_state_size;
	
	double wheel_diameter;
	double wheel_distance;
	double center_to_lens;

	int descr_size; 
	
	double min_dist;

	double focal_lenght_x;
	double focal_lenght_y;	
	
	int image_size_x;
	int image_size_y;

	double image_scale;

	double image_center_x;
	double image_center_y;		

	double removal_gain;
	double removal_threshold;
	
	double init_std_lin_vel;
	double init_std_ang_vel;
	double init_inv_depth;
	double init_std_inv_depth;
		
	double std_lin_acc;
	double std_ang_acc;
	double std_imu_vel;
	double std_wheel_vel;
	double std_arm;	
	
	double std_pixel;
		
} Stuff;

// ------------------------------------------------------------
//                      Function Declaration
// ------------------------------------------------------------
	
static int cmp_func(const void* _a, const void* _b, void* userdata);

void matchingORB(const CvSeq*, const CvSeq*, vector<int>&);

double compareSURFDescriptors(const float*, const float*, double, int);

int naiveNearestNeighbor(const float*, int, const CvSeq*, const CvSeq*);
                      
void findPairs(const CvSeq*, const CvSeq*, const CvSeq*, const CvSeq*, vector<int>&);

void flannFindPairs(const CvSeq*, const CvSeq*, const CvSeq*, const CvSeq*, vector<int>&);

// ------------------------------------------------------------
//                         CLASSES
// ------------------------------------------------------------
 
class Slam{

	//cv::SURF *surf;

	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Publisher pub_data;
		
	ros::Subscriber sub_IMU;
	ros::Subscriber sub_Odo;
	ros::Subscriber sub_Feat;
	ros::Timer 			sub_Pub;	
		
	tf::TransformBroadcaster br;
	
	image_transport::ImageTransport *it;
		
	image_transport::Subscriber sub_Img_L, sub_Img_R;	
	image_transport::Publisher pubIL;
	
	sensor_msgs::PointCloud data_points;
	
	State    s;
	Feat_filter im;
	Stuff 	 params;
	
	CvSURFParams SURFparams;

	bool first_image;
	bool first_imu;	
	bool first_odo;
	bool update_imu;
	bool update_odo;
	
	bool got_image;		
	bool got_odo;
	int visible_and_detected;
	
	int sz;
	int sz_w;
	
	cv::Mat imleft;
	cv::Mat imleft_graph;
	
	double tic_tac;
	
	double time_prev_imu, time_now_imu;
	double time_prev_odo, time_now_odo;

	int descr_size;
	
	Quaterniond q_imu0_earth;
	Quaterniond q_inew0_imu0;
	Quaterniond q_imu_inew;	
	Quaterniond q_inew0_inew;	

	Quaterniond q_old;	
	Quaterniond q_new;	
	
	imu_w w_imu;
	
	odometry odo_new;
	odometry odo_prev;
	odometry odo_dif;	
	double 	 odo_arm;

	
	int feat_removed;
	int feat_added;

public:
	
	Slam(ros::NodeHandle n);
	
 ~Slam();

	Matrix<double,3,4> quaternion_dRqM_by_dq(double w, double x, double y, double z, Vector3d M);

	void ekf(const double dt);

	void predict(const double dt);
	
	void update(vector<int>& pairs);
	
	void insertFeatures();
	
	void removeFeatures();
	
	void writeLandmarks();
	
	void extractFeatures();
	
	void initializeState();

	void publish();

	vector<int> matching();

	void callbackPublish(const ros::TimerEvent &event);

	void callbackOdo(const raposang_msgs::RaposaOdometry &odo_data);

	void callbackIMU(const sensor_msgs::Imu &imu_data);
	
	void callbackImage(const sensor_msgs::ImageConstPtr &img_msg);

	void callbackFeatures(const raposang_msgs::RaposaFeatures &feat_msg);

	void normalizeQuaternionInState();
	
	void rosSpin();
	
private:
		
}; 

// EOF
