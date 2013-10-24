#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <time.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CompressedImage.h>
#include <iostream>
#include <iterator>
#include <fstream>

char filename[255];
std::string path;
sensor_msgs::CompressedImage img;
cv::Mat image;
using namespace std;


void IMcallback(const sensor_msgs::CompressedImage& image_msg){
    img=image_msg;
}

bool save_image(std_srvs::Empty::Request &dummyr, std_srvs::Empty::Response &dummyq){
    if (!img.data.empty()) {
        time_t now = time(0);
        struct tm* tm = localtime(&now);
        sprintf(filename,"%s%02d-%02d-%02d-%02d-%02d-%02d.jpeg",path.c_str(),(tm->tm_year + 1900),(tm->tm_mon + 1),tm->tm_mday,tm->tm_hour, tm->tm_min, tm->tm_sec);
        ofstream FILE(filename, ios::out | ofstream::binary);
        std::copy(img.data.begin(), img.data.end(), ostreambuf_iterator<char>(FILE));
        FILE.close();

        ROS_INFO("Image Saved");
        return true;

    } else {
        ROS_WARN("Couldn't save image, no data!");
        return false;
    }
}

void TIMERcallback(const ros::TimerEvent& event){
    if (!img.data.empty()) {
        time_t now = time(0);
        struct tm* tm = localtime(&now);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        sprintf(filename,"%s%02d-%02d-%02d-%02d-%02d-%02d-%ld.jpeg",path.c_str(),(tm->tm_year + 1900),(tm->tm_mon + 1),tm->tm_mday,tm->tm_hour, tm->tm_min, tm->tm_sec, tv.tv_usec);
        ofstream FILE(filename, ios::out | ofstream::binary);
        std::copy(img.data.begin(), img.data.end(), ostreambuf_iterator<char>(FILE));
        FILE.close();

        ROS_INFO("Image Saved");
    } else {
        ROS_WARN("Couldn't save image, no data!");
    }
}

int main(int argc, char** argv){
    std::string img_topic;
    ros::init(argc, argv, "save_image", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    nh.param<std::string>("topic", img_topic, "/usb_cam/image_raw/compressed");
    nh.param<std::string>("path", path, "/home/quadbase/ros/quad_ros/quad_common/quad_save_image/images/");
    ros::ServiceServer service = nh.advertiseService("save_image", save_image);
    ros::Subscriber sub = nh.subscribe(img_topic, 10, IMcallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), TIMERcallback);

    ros::spin();
}
