#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <kinect_driver.h>
#include <rosbag/bag.h>
#include <rosbag/stream.h>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <highgui.h>
#include <time.h>

ros::Publisher pub_pointcloud;
//ros::Publisher pub_image;
//image_transport::Publisher pub_image;
//! to do camera
image_transport::CameraPublisher pub_image_cam;

int
main (int argc, char** argv)
{
	
	double frame_rate;
	bool record_bag, record_video;

	// Initialize ROS
	ros::init (argc, argv, "kinect_driver_node");
	ros::NodeHandle node_handler;
	ros::NodeHandle nh("~");

	nh.param("frame_rate",frame_rate,30.0);
	nh.param("record_bag",record_bag,false);
	nh.param("record_video",record_video,false);
	
	
	image_transport::ImageTransport img_trans(node_handler);

	// Create a ROS publisher for the output point cloud
	//! por no formato do point cloud
	pub_pointcloud = node_handler.advertise< sensor_msgs::PointCloud2 > ("pointcloud", 1);
	//pub_image = img_trans.advertise("image", 1);
	pub_image_cam = img_trans.advertiseCamera("image",  1);
	
	//rosbag::Bag bag;
	//bag.setCompression(rosbag::compression::BZ2);
	//if(record_bag){
	//	bag.open("/home/ist_isr/Desktop/pointcloud.bag", rosbag::bagmode::Write);}

	ros::Rate loop_rate(frame_rate);
	
	try {
		
		KinectDriver kinect(node_handler);
		kinect.StartGenerating();
		ROS_INFO("Kinect driver node initialized!");
		
		cv_bridge::CvImagePtr cv_ptr;
		cv::VideoWriter recorder;
		time_t rawtime;
		//cv::namedWindow("video",CV_WINDOW_AUTOSIZE + CV_WINDOW_KEEPRATIO + CV_GUI_EXPANDED);
		time (&rawtime);
		std::string file_name="/home/cobot/Desktop/kinect_video_";
		file_name = file_name + ctime (&rawtime) + ".avi";
		if(record_video){
			
			recorder.open(file_name, CV_FOURCC('D', 'I', 'V', 'X'), frame_rate, cvSize(640,480), true);
			if( !recorder.isOpened() ) {
                ROS_INFO("VideoWriter failed to open!\n");
			}
		}

		while (ros::ok())
		{
			ros::spinOnce();
			if(pub_pointcloud.getNumSubscribers() > 0 || pub_image_cam.getNumSubscribers() > 0 ){
				kinect.SetNewData();
				pub_pointcloud.publish(kinect.GetPointCloud());
				//pub_image.publish(kinect.GetImage());
				pub_image_cam.publish(kinect.GetImage(), kinect.GetCameraInfo());
				if(record_video){
					cv_ptr = cv_bridge::toCvCopy(kinect.GetImage(), sensor_msgs::image_encodings::BGR8);
					recorder << cv_ptr->image;
					//cv::imshow("video", cv_ptr->image);
				}
				//if(record_bag){
				//	bag.write("pointcloud", ros::Time::now(), kinect.GetPointCloud());}
			}
			loop_rate.sleep();
		}
	}
	catch (std::string str_err){
		ROS_ERROR("%s",str_err.c_str());
	}
	
	//if(record_bag){
	//	bag.close();}
	return 0;
}
