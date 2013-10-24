#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "mav_msgs/Height.h"

std::string global_frame_id;
std::string footprint_frame_id;
std::string stabilized_frame_id;
std::string base_frame_id;
std::string odom_frame_id;

std::string imu_topic;
std::string height_topic;

tf::TransformBroadcaster *g_transform_broadcaster;


#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void addTransform(std::vector<geometry_msgs::TransformStamped>& transforms, const tf::StampedTransform& tf)
{
  transforms.resize(transforms.size()+1);
  tf::transformStampedTFToMsg(tf, transforms.back());
}

/*void sendTransform(geometry_msgs::Pose const &pose, const std_msgs::Header& header, std::string child_frame_id = "")
{
  std::vector<geometry_msgs::TransformStamped> transforms;

  tf::StampedTransform tf;
  tf.frame_id_ = header.frame_id;
  if (!g_frame_id.empty()) tf.frame_id_ = g_frame_id;
  tf.stamp_ = header.stamp;
  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Point position;
  tf::pointMsgToTF(pose.position, position);

  // position intermediate transform (x,y,z)
  if( !g_position_frame_id.empty() && child_frame_id != g_position_frame_id) {
    tf.child_frame_id_ = g_position_frame_id;
    tf.setOrigin(tf::Vector3(position.x(), position.y(), position.z() ));
    tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    addTransform(transforms, tf);
  }

  // footprint intermediate transform (x,y,yaw)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_footprint_frame_id) {
    tf.child_frame_id_ = g_footprint_frame_id;
    tf.setOrigin(tf::Vector3(position.x(), position.y(), 0.0));
    tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
    addTransform(transforms, tf);

    yaw = 0.0;
    position.setX(0.0);
    position.setY(0.0);
    tf.frame_id_ = g_footprint_frame_id;
  }

  // stabilized intermediate transform (z)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_stabilized_frame_id) {
    tf.child_frame_id_ = g_stabilized_frame_id;
    tf.setOrigin(tf::Vector3(0.0, 0.0, position.z()));
    tf.setBasis(tf::Matrix3x3::getIdentity());
    addTransform(transforms, tf);

    position.setZ(0.0);
    tf.frame_id_ = g_stabilized_frame_id;
  }

  // base_link transform (roll, pitch)
  tf.child_frame_id_ = child_frame_id;
  tf.setOrigin(position);
  tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
  addTransform(transforms, tf);

  g_transform_broadcaster->sendTransform(transforms);

}*/

void heightCallback(const mav_msgs::Height& msg){
    std::vector<geometry_msgs::TransformStamped> transforms;

    // stabilized intermediate transform (z)
    tf::StampedTransform tf;
    tf.stamp_ = msg.header.stamp;
    tf.frame_id_ = footprint_frame_id;
    tf.child_frame_id_ = stabilized_frame_id;
    tf.setOrigin(tf::Vector3(0.0, 0.0, msg.height));
    tf.setBasis(tf::Matrix3x3::getIdentity());

    g_transform_broadcaster->sendTransform(tf);
}

void imuCallback(sensor_msgs::Imu const &imu) {
  std::vector<geometry_msgs::TransformStamped> transforms;

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imu.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);

  tf::StampedTransform tf;
  tf.stamp_ = imu.header.stamp;

  // footprint intermediate transform (x,y,yaw)
  tf.frame_id_ = global_frame_id;
  tf.child_frame_id_ = footprint_frame_id;
  tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
  addTransform(transforms, tf); // Add transform to the buffer

  // base_link transform (roll, pitch)
  tf.frame_id_ = stabilized_frame_id;
  tf.child_frame_id_ = base_frame_id;
  tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0.0));
  addTransform(transforms, tf); // Add transform to the buffer

  g_transform_broadcaster->sendTransform(transforms);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "message_pub_tf");

  ros::NodeHandle nh("~");
  nh.param<std::string>("global_frame", global_frame_id, "map");
  nh.param<std::string>("footprint_frame", footprint_frame_id, "base_footprint");
  nh.param<std::string>("stabilized_frame", stabilized_frame_id, "base_stabilized");
  nh.param<std::string>("base_frame", base_frame_id, "base_link");
  nh.param<std::string>("odom_frame", odom_frame_id, "odom");

  nh.getParam("imu_topic", imu_topic);
  nh.getParam("height_topic", height_topic);

  g_transform_broadcaster = new tf::TransformBroadcaster;

  ros::Subscriber sub1, sub2;
  if (!imu_topic.empty())      sub1 = nh.subscribe(imu_topic, 10, &imuCallback);
  if (!height_topic.empty())   sub2 = nh.subscribe(height_topic, 10, &heightCallback);

  if (!sub1 || !sub2) {
    ROS_FATAL("Params imu_topic, height_topic are empty... nothing to do for me!");
    return 1;
  }

  ros::spin();
  delete g_transform_broadcaster;
  return 0;
}

