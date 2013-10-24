#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <time.h>
#include <std_srvs/Empty.h>



char filename[255];
std::string path;
cv::Mat image;

void IMcallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info){

  try
  {
    image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
  } catch(cv_bridge::Exception)
  {
    ROS_ERROR("Unable to convert %s image to bgr8", image_msg->encoding.c_str());
    return;
  }
  
    /*if (!image.empty()) {
      time_t now = time(0);
      struct tm* tm = localtime(&now);
      sprintf(filename, "%02d-%02d-%02d-%02d-%02d-%02d.jpeg",(tm->tm_year + 1900),(tm->tm_mon + 1),tm->tm_mday,tm->tm_hour, tm->tm_min, tm->tm_sec);
      cv::imwrite(filename, image);
      ROS_INFO("Image Saved");
      
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }*/
}

bool save_image(std_srvs::Empty::Request &dummyr, std_srvs::Empty::Response &dummyq){
	
  if (!image.empty()) {
      time_t now = time(0);
      struct tm* tm = localtime(&now);
      sprintf(filename,"/home/quadbase/ros/quad_ros/quad_common/quad_save_image/images/%02d-%02d-%02d-%02d-%02d-%02d.jpeg",(tm->tm_year + 1900),(tm->tm_mon + 1),tm->tm_mday,tm->tm_hour, tm->tm_min, tm->tm_sec);
      cv::imwrite(filename, image);
      ROS_INFO("Image Saved");
      return true;
      
    } else {
      ROS_WARN("Couldn't save image, no data!");
      return false;
    }
}

int main(int argc, char** argv)
{
  std::string img_topic;
  ros::init(argc, argv, "save_image", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  nh.param<std::string>("topic", img_topic, "/usb_cam/image_raw/compressed");
  nh.param<std::string>("path", path, "/home/quadbase/ros/quad_ros/quad_common/quad_save_image/images/");
  ros::ServiceServer service = nh.advertiseService("save_image", save_image);
  //ros::Subscriber sub = nh.subscribe(img_topic, 10, IMcallback);
  image_transport::CameraSubscriber sub = it.subscribeCamera(img_topic, 1, &IMcallback);

  ros::spin();
}
