#include <XnCppWrapper.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>


#ifndef __KINECT_DRIVER_H__
#define __KINECT_DRIVER_H__


typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;


//Modes suported by the kinect
#define VGA_MODE 0

class KinectDriver
{
	
	//Intrinsec Parameters
	double cu;
	double cv;
	double fu;
	double fv;
	
	xn::Context context;
	
	//All Generatores nodes
	xn::DepthGenerator depth;
	xn::ImageGenerator image;
	
	XnMapOutputMode mapMode;
	
	//All the infornation produc by the driver
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;
	sensor_msgs::Image rgb_image;
	sensor_msgs::CameraInfo rgb_image_info;
	camera_info_manager::CameraInfoManager *cameraInfo;
	
	
public:	
	KinectDriver(ros::NodeHandle nh, int mode=VGA_MODE);
	
	~KinectDriver();
	
	void SetVGAMode();
	
	void StartGenerating();
	
	void UpdateNodes();
	
	void SetData();
	
	void SetNewData();
	
	sensor_msgs::Image GetImage();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPointCloud();

	void SetIntrinsecParam(double _cu=3.2894272028759258e+02,
									double _cv=2.6748068171871557e+02,
									double _fu=5.2921508098293293e+02,
									double _fv=5.2556393630057437e+02);
									
	sensor_msgs::CameraInfo GetCameraInfo();
	
	
private:
	void CreatAllGenerators();
	
	void SetMapOutputParametres(const XnUInt32& nXRes, const XnUInt32& nYRes, const XnUInt32& nFPS);

	void ChangeViewPoint();

};

#endif //__KINECT_DRIVER_H__

