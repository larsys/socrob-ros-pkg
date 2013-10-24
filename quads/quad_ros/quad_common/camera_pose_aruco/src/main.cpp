//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PoseStamped.h>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "MarkersConfig.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include <tf/transform_broadcaster.h>


using namespace cv;
using namespace aruco;



double TheMarkerSize=0.057;
MarkerDetector MDetector;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
geometry_msgs::PoseStamped t_camera_pose;
geometry_msgs::PointStamped t_pixel_disp;
//tf::TransformBroadcaster t_br_camera_pose;

std::string _camera_image;



bool want2watch=0;
string watch="";


//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub_img;
ros::Publisher pub_pose, pub_pose_all;
ros::Publisher pub_pixels_disp;
MarkersConfig Markers_Sizes;
geometry_msgs::PoseStamped camera_pose_mean;


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing




    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }


    ////////////// Começa aqui (ArUco):
    TheInputImage = cv_ptr->image;
    MDetector.detect2(TheInputImage,TheMarkers,TheCameraParameters,Markers_Sizes.msize,Markers_Sizes.mposition, false);

    TheInputImage.copyTo(TheInputImageCopy);
    for (unsigned int i=0;i<TheMarkers.size();i++) {
//        ROS_INFO("Cenas: " << TheMarkers[i]<<endl);
        TheMarkers[i].draw_size(TheInputImageCopy,Scalar(0,0,255),1);
    }

//    std::cout << "Has value - " << std::boolalpha <<  TheCameraParameters.isValid() << endl;

    if (  TheCameraParameters.isValid())
        for (unsigned int i=0;i<TheMarkers.size();i++) {
            CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
            CvDrawingUtils::draw3dCube_z(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
            cv::circle(TheInputImageCopy,TheMarkers[i].marker_center_img,5, Scalar(0,255,255,255),5);
        }

    cv_ptr->image = TheInputImageCopy;
    /////////////////////// Acaba aqui <-





 
    //Display the image using OpenCV
    if(want2watch){
        cv::imshow(WINDOW, cv_ptr->image);
        //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
        cv::waitKey(3);
        /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor in main().
        */
        //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub_img.publish(cv_ptr->toImageMsg());
    }

    ////////////////////// 2ª parte começa aqui:

//    if (TheMarkers.size() != 1) {
//        if (TheMarkers.size() > 1) cout << ros::Time::now() << " Ahhhhhh <=> " << TheMarkers.size() << endl;
//        return;
//    }



    cv::Mat Rotation_cam(3,3,CV_32FC1);
    cv::Mat Position_cam(3,1,CV_32FC1);


    // to keep the mean
    geometry_msgs::PoseStamped cam_pose_aux;
    cam_pose_aux.pose.position.x=cam_pose_aux.pose.position.y=cam_pose_aux.pose.position.z=0.0;
    cam_pose_aux.pose.orientation.w=cam_pose_aux.pose.orientation.x=cam_pose_aux.pose.orientation.y=cam_pose_aux.pose.orientation.z=0.0;

    if (  TheCameraParameters.isValid()){
        for (unsigned int i=0;i<TheMarkers.size();i++) {
            // Ts * Rodrigues(Rs)
            cv::Rodrigues( TheMarkers[i].Rvec,Rotation_cam );
            Position_cam = -TheMarkers[i].Tvec.t() * Rotation_cam;
//            Position_cam = -Rotation_cam.t() * TheMarkers[i].Tvec;
            Position_cam = Position_cam.t();



            /////////////////////// To quaternion: (The inverse of the rotation)

            double qw = sqrt(1 + Rotation_cam.at<double>(0,0) + Rotation_cam.at<double>(1,1) + Rotation_cam.at<double>(2,2))/2;
        //    double qx = (Rotation_cam.at<double>(2,1) - Rotation_cam.at<double>(1,2))/( 4 *qw);
        //    double qy = (Rotation_cam.at<double>(0,2) - Rotation_cam.at<double>(2,0))/( 4 *qw);
        //    double qz = (Rotation_cam.at<double>(1,0) - Rotation_cam.at<double>(0,1))/( 4 *qw);
            double qx = (Rotation_cam.at<double>(1,2) - Rotation_cam.at<double>(2,1))/( 4 *qw);
            double qy = (Rotation_cam.at<double>(2,0) - Rotation_cam.at<double>(0,2))/( 4 *qw);
            double qz = (Rotation_cam.at<double>(0,1) - Rotation_cam.at<double>(1,0))/( 4 *qw);

            /////////////////////

            // To use in mean
            cam_pose_aux.pose.orientation.w  += qw;
            cam_pose_aux.pose.orientation.x  += qx;
            cam_pose_aux.pose.orientation.y  += qy;
            cam_pose_aux.pose.orientation.z  += qz;
            cam_pose_aux.pose.position.x     += Position_cam.at<double>(0,0);
            cam_pose_aux.pose.position.y     += Position_cam.at<double>(0,1);
            cam_pose_aux.pose.position.z     += Position_cam.at<double>(0,2);
            /////////////


            t_camera_pose.header.frame_id="quad_cam" +  boost::lexical_cast<string>(TheMarkers[i].id) ;
            t_camera_pose.header.stamp = ros::Time::now();
            t_camera_pose.pose.position.x = Position_cam.at<double>(0,0);
            t_camera_pose.pose.position.y = Position_cam.at<double>(0,1);
            t_camera_pose.pose.position.z = Position_cam.at<double>(0,2);
            t_camera_pose.pose.orientation.x = qx;
            t_camera_pose.pose.orientation.y = qy;
            t_camera_pose.pose.orientation.z = qz;
            t_camera_pose.pose.orientation.w = qw;


            // isto não está a funcionar!!!

            if (  TheCameraParameters.isValid()){
                double y_sum=0, x_sum=0;
                for (uint i=0; i < TheMarkers.size(); i++){
                    x_sum+= TheMarkers[i].marker_center_img.x;
                    y_sum+= TheMarkers[i].marker_center_img.y;
                }
                x_sum /= TheMarkers.size();
                y_sum /= TheMarkers.size();

                t_pixel_disp.header.frame_id = "Displacement_in_img";
                t_pixel_disp.header.stamp = ros::Time::now();
                t_pixel_disp.point.x = (double) x_sum/TheMarkers.size() - (double) TheCameraParameters.CamSize.width/2;
                t_pixel_disp.point.y = (double) y_sum/TheMarkers.size() - (double)  TheCameraParameters.CamSize.height/2;
            }

            ROS_INFO("%lf  %lf",t_pixel_disp.point.x, t_pixel_disp.point.y);

            pub_pixels_disp.publish(t_pixel_disp);

            ///////////// e acaba aqui...

            pub_pose_all.publish(t_camera_pose);


            static tf::TransformBroadcaster t_br_camera_pose;
            tf::Transform tf_quad;

            // adds the translation of each marker
            tf_quad.setOrigin( tf::Vector3(t_camera_pose.pose.position.x+Markers_Sizes.mposition.at(TheMarkers[i].id).x,
                                           t_camera_pose.pose.position.y+Markers_Sizes.mposition.at(TheMarkers[i].id).y,
                                           t_camera_pose.pose.position.z+Markers_Sizes.mposition.at(TheMarkers[i].id).z) );
//            tf_quad.setOrigin( tf::Vector3(t_camera_pose.pose.position.x,
//                                           t_camera_pose.pose.position.y,
//                                           t_camera_pose.pose.position.z) );
            tf_quad.setRotation( (const tf::Quaternion&) t_camera_pose.pose.orientation);

            t_br_camera_pose.sendTransform(tf::StampedTransform(tf_quad, ros::Time::now(), "ref_marker", boost::lexical_cast<string>(TheMarkers[i].id)));


            // debug:
            ROS_INFO_STREAM("published id="<<TheMarkers[i].id);
        }


        cam_pose_aux.header.frame_id="quad_cam_mean_no_detect";
        cam_pose_aux.header.stamp = ros::Time::now();

        if (TheMarkers.size() > 0){
            // Calculate mean position

            cam_pose_aux.header.frame_id="quad_cam_mean";
            cam_pose_aux.pose.orientation.w  /= TheMarkers.size();
            cam_pose_aux.pose.orientation.x  /= TheMarkers.size();
            cam_pose_aux.pose.orientation.y  /= TheMarkers.size();
            cam_pose_aux.pose.orientation.z  /= TheMarkers.size();
            cam_pose_aux.pose.position.x     /= TheMarkers.size();
            cam_pose_aux.pose.position.y     /= TheMarkers.size();
            cam_pose_aux.pose.position.z     /= TheMarkers.size();

            camera_pose_mean = cam_pose_aux;


            static tf::TransformBroadcaster t_br_camera_pose_mean;
            tf::Transform tf_quad_mean;

            // adds the translation of each marker
            tf_quad_mean.setOrigin( tf::Vector3(cam_pose_aux.pose.position.x,
                                           cam_pose_aux.pose.position.y,
                                           cam_pose_aux.pose.position.z ));
            tf_quad_mean.setRotation( (const tf::Quaternion&) t_camera_pose.pose.orientation);

            t_br_camera_pose_mean.sendTransform(tf::StampedTransform(tf_quad_mean, ros::Time::now(), "ref_marker", "cam_pose_mean"));


        }

        pub_pose.publish(camera_pose_mean);

    }
}
 


int main(int argc, char **argv)
{

    ros::init(argc, argv, "camera_pose_aruco");
    ros::NodeHandle nh("~");

    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    std::string _markers_config_file;
    std::string _camara_calib_file;


    nh.param<std::string>("markers_config_file",_markers_config_file,"markers_sizes_aruco.txt");
    nh.param<std::string>("camara_calib_file",_camara_calib_file,"cam_parameters_aruco.txt");
    nh.param<std::string>("image",_camera_image,"usb_cam/image_raw");

    ROS_INFO_STREAM(_markers_config_file << " " << _camara_calib_file << " " << _camera_image);

    Markers_Sizes.parse_from_file(_markers_config_file,0.057);
    TheCameraParameters.readFromFile(_camara_calib_file);

    if(!TheCameraParameters.isValid())
        cout << "Warning! The file with camera parameters was NOT detected" << endl;
    else
        cout << "The file with camera parameters was correctly read." << endl << "Starting to publish pose_cam" << endl;

    MDetector.setThresholdParams(7, 7);
    MDetector.setCornerRefinementMethod(MarkerDetector::LINES);


    if(watch.compare("true")){ want2watch=true;}

    if(want2watch)
        cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

    image_transport::Subscriber sub = it.subscribe(_camera_image, 1, imageCallback);

    if(want2watch)
        cv::destroyWindow(WINDOW);

    if(want2watch)
        pub_img = it.advertise("image_processed", 1);

    pub_pose_all = nh.advertise<geometry_msgs::PoseStamped>("pose_cam_all",15);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose_cam",5);
    pub_pixels_disp = nh.advertise<geometry_msgs::PointStamped>("pixel_disp",5);

    ros::spin();
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");


    return 0;
}
