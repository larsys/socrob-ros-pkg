#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

std::string odometry_topic;
std::string imu_topic;

std::string footprint_frame_id;
std::string stabilized_frame_id;
std::string base_frame_id;
std::string global_frame_id;

tf::TransformBroadcaster *transform_broadcaster;

#ifndef TF_MATRIX3x3_H
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

/*void odomCallback(nav_msgs::Odometry const &odometry) {
  sendTransform(odometry.pose.pose, odometry.header, odometry.child_frame_id);
}*/


void imuCallback(sensor_msgs::Imu const &imu) {
	std::vector<geometry_msgs::TransformStamped> transforms;
	
	tf::StampedTransform transform;
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(imu.orientation, orientation);
  	btScalar yaw, pitch, roll;
	tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  
	// Stabilized transform (z, yaw)
 	transform.frame_id_ = global_frame_id;
	transform.child_frame_id_ = stabilized_frame_id;
	transform.stamp_ = imu.header.stamp;
	
    transform.setOrigin(tf::Vector3(0.0, 0.0, 1.0));
    transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
    addTransform(transforms, transform); // Add transform to the buffer
  
	// Base transform (roll, pitch)
	transform.frame_id_ = stabilized_frame_id;
	transform.child_frame_id_ = base_frame_id;
	transform.stamp_ = imu.header.stamp;
  
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0.0));
	addTransform(transforms, transform); // Add transform to the buffer

	// Send both Transforms
	transform_broadcaster->sendTransform(transforms);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "message_to_tf");
	ros::NodeHandle nh("~");

	//nh.getParam("odometry_topic", odometry_topic);
	
	if (!nh.getParam("imu_topic", imu_topic)){
      	ROS_FATAL("Params 'imu_topic' is empty!");
		return 1;
    }
    
    nh.param<std::string>("global_frame_id", global_frame_id, "map");
    nh.param<std::string>("footprint_frame_id", footprint_frame_id, "base_footprint");
    nh.param<std::string>("stabilized_frame_id", stabilized_frame_id, "base_stabilized");
    nh.param<std::string>("base_frame_id", base_frame_id, "base_link");
	
	transform_broadcaster = new tf::TransformBroadcaster;

	ros::Subscriber sub1;
	sub1 = nh.subscribe(imu_topic, 10, &imuCallback);
	//sub2 = nh.subscribe(odometry_topic, 10, &odomCallback);

	ros::spin();
	delete transform_broadcaster;
	return 0;
}
