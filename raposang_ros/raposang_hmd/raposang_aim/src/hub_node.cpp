/***********/
/* Include */
/***********/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "raposang_msgs/PanasonicCameraControl.h"

/****************/
/* Global value */
/****************/

float x, y;

/*************/
/* Functions */
/*************/

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber axis_sub;
  float x, y;
  
//  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {

    x = 0;
    y = 0;

    image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
    axis_sub = nh_.subscribe("input_joy", 1, &ImageConverter::axis, this);
    
    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void axis(const raposang_msgs::PanasonicCameraControl& data) 
  {
    x = data.x;
    y = data.y;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(x, y), 10, CV_RGB(255,0,0));

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
//    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
