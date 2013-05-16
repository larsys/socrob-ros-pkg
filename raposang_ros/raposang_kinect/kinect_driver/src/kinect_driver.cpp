#include <kinect_driver.h>
#include <XnCppWrapper.h>
#include <limits>
#include <time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <openni_camera/openni_image_rgb24.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>


KinectDriver::KinectDriver(ros::NodeHandle nh, int mode){
	
	cameraInfo=(new camera_info_manager::CameraInfoManager(nh,"kinect","package://kinect_driver/config/cal.yaml") );
	rgb_image_info = cameraInfo->getCameraInfo();
	rgb_image_info.header.frame_id="kinect_rgb_optical_frame";
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	nRetVal = context.Init();
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("initiating Context failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
	CreatAllGenerators();

	//To choose the mode in wich the kinect will run
	switch ( mode ) {
		case VGA_MODE:
			SetVGAMode();
		break;
		//To add a new mode of kinect, one can just add the same lines of code like before but with the
		//respective new mode. It is also better to define a macro and a 'Set' function for the mode in
		//order to understand better the code
	}
	
	ChangeViewPoint();
	
	SetIntrinsecParam();

}
	
KinectDriver::~KinectDriver(){
	
	//context.Release();
	context.Shutdown();
	
}
	
void KinectDriver::SetVGAMode(){
	
	SetMapOutputParametres(XN_VGA_X_RES, XN_VGA_Y_RES, 30);
	
	pointcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>() );
	pointcloud->header.frame_id="kinect_rgb_optical_frame";
	pointcloud->is_dense=false;
	pointcloud->width=XN_VGA_X_RES;
	pointcloud->height=XN_VGA_Y_RES;
	pointcloud->points.resize(XN_VGA_X_RES*XN_VGA_Y_RES);
	
	//~ rgb_image = boost::make_shared<sensor_msgs::Image > ();
	rgb_image.header.frame_id="kinect_rgb_optical_frame";
	rgb_image.width=XN_VGA_X_RES;
	rgb_image.height=XN_VGA_Y_RES;
	rgb_image.step = XN_VGA_X_RES*3;
	rgb_image.encoding = sensor_msgs::image_encodings::RGB8;
	rgb_image.data.resize(XN_VGA_Y_RES*XN_VGA_X_RES*3);
	//uint8 is_bigendian;
	
}
	
void KinectDriver::StartGenerating(){
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	nRetVal = context.StartGeneratingAll();
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Starting Generating failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
}
	
void KinectDriver::UpdateNodes(){
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	nRetVal = context.WaitOneUpdateAll(depth);
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Updating Nodes failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
}

void KinectDriver::SetData(){
	
	rgb_image.header.stamp = pointcloud->header.stamp = rgb_image_info.header.stamp = ros::Time::now();
	
	unsigned color_step, color_skip;
	color_step = 3 * rgb_image.width / pointcloud->width;
    color_skip = 3 * (rgb_image.height / pointcloud->height - 1) * rgb_image.width;
	
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	
	//Get the depth image
	const XnDepthPixel* pDepthMap = depth.GetDepthMap();
	
	//Get the RGB image and fill the image msg
	boost::shared_ptr<xn::ImageMetaData> image_meta_data( new xn::ImageMetaData() );
	image.GetMetaData(*image_meta_data);
	openni_wrapper::ImageRGB24 image_openni_wrapper(image_meta_data);
	image_openni_wrapper.fillRGB (rgb_image.width, rgb_image.height, &rgb_image.data[0], rgb_image.step);
	const uint8_t* rgb_buffer = &rgb_image.data[0];
	
	//Fill the pointcloud msg
	double depth_dist;
	int color_idx=0;
	RGBValue color;
	pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = pointcloud->begin ();
	for(uint v=0; v<mapMode.nYRes; v++, color_idx+=color_skip  ){ 
		for(uint u=0; u<mapMode.nXRes; u++, ++pt_iter, color_idx+=color_step){
			
			pcl::PointXYZRGB& pt = *pt_iter;
			
			color.Red = rgb_buffer[color_idx];
			color.Green = rgb_buffer[color_idx+1];
			color.Blue = rgb_buffer[color_idx+2];
			color.Alpha = 0;
			pt.rgb = color.float_value;

			if((depth_dist = pDepthMap[v * pointcloud->width + u]*0.001)!=0){
				pt.x = (u-cu)*depth_dist/fu;
				pt.y = (v-cv)*depth_dist/fv;
				pt.z = depth_dist;
			}
			else{
				pt.x = pt.y = pt.z = bad_point;
			}
		}
	}
	

}
	
void KinectDriver::SetNewData(){
	
	UpdateNodes();
	
	SetData();
	
}

sensor_msgs::Image KinectDriver::GetImage(){
	return rgb_image;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectDriver::GetPointCloud(){
	return pointcloud;
}

void KinectDriver::SetIntrinsecParam(double _cu, double _cv, double _fu, double _fv){
	cu=_cu;
	cv=_cv;
	fu=_fu;
	fv=_fv;
}

sensor_msgs::CameraInfo KinectDriver::GetCameraInfo(){
	return rgb_image_info;
}

inline void KinectDriver::CreatAllGenerators(){
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	nRetVal = depth.Create(context);
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Creating a Deph Generator node failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
	nRetVal = image.Create(context);
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Creating an Image Generator node failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
	//To add a new node of the OpenNI lib, one just add the same lines of code like before but with the
	//respective new Generator classe
}
	
inline void KinectDriver::SetMapOutputParametres(const XnUInt32& nXRes, const XnUInt32& nYRes, const XnUInt32& nFPS){
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	mapMode.nXRes = nXRes;
	mapMode.nYRes = nYRes;
	mapMode.nFPS = nFPS;
	
	nRetVal = depth.SetMapOutputMode(mapMode);
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Setting Map Output Mode on Depth Genetator failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
	nRetVal = image.SetMapOutputMode(mapMode);
	if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Setting Map Output Mode on Image Genetator failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
}

inline void KinectDriver::ChangeViewPoint(){
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(image); 
    if (nRetVal != XN_STATUS_OK){
		std::string str_err ("Getting and setting AlternativeViewPoint of Depth Generator failed: ");
		str_err += xnGetStatusString(nRetVal); //str_err += "\n";
		throw str_err;
	}
	
}


