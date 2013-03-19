#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

geometry_msgs::PoseStamped slam;
sensor_msgs::Imu imu;

bool used_imu = true;
bool used_slam = true;

geometry_msgs::PoseStamped current_pose;

void imu_data (sensor_msgs::Imu data)
{
	imu = data;
	used_imu = false;

	//
	double yaw,pitch,roll,tmp;
	tf::Quaternion q;
	tf::quaternionMsgToTF(data.orientation, q);
	tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
	ROS_INFO("imu yaw:%f pitch:%f roll:%f ", yaw,pitch,roll);
	//
}

void slam_data (geometry_msgs::PoseStamped data)
{
	slam = data;
	//
	double yaw,pitch,roll,tmp;
	tf::Quaternion q;
	tf::quaternionMsgToTF(data.pose.orientation, q);
	tf::Matrix3x3(q).getEulerYPR(yaw, tmp, tmp);
	ROS_INFO("slam yaw:%f", yaw);
	//
	used_slam = false;
}

geometry_msgs::PoseStamped join_data(geometry_msgs::PoseStamped slam, sensor_msgs::Imu imu)
{
	double yaw,pitch,roll,tmp;
	tf::Quaternion q;
	geometry_msgs::PoseStamped final;

	//If we have unused slam data we get the position and the yaw
	//If we don't, we get the last pose
	if (!used_slam)
	{
		final = slam;
		tf::quaternionMsgToTF(final.pose.orientation, q);
		tf::Matrix3x3(q).getEulerYPR(yaw, tmp, tmp);
	}
	else
	{
		final = current_pose;
		tf::quaternionMsgToTF(final.pose.orientation, q);
		tf::Matrix3x3(q).getEulerYPR(yaw, tmp, tmp);
	}

	//If we have imu data, we use it for pitch and roll only
	if (!used_imu)
	{
		tf::quaternionMsgToTF(imu.orientation, q);
		tf::Matrix3x3(q).getEulerYPR(tmp, pitch, roll);
	}
	else
	{
		final = current_pose;
		tf::quaternionMsgToTF(final.pose.orientation, q);
		tf::Matrix3x3(q).getEulerYPR(tmp, pitch, roll);
	}

	//get the most recent time stamp
	if (imu.header.stamp>slam.header.stamp)
		final.header.stamp = imu.header.stamp;

	//update the quaternion with the angles we got
	q.setRPY(roll,pitch,yaw);

	//transform back to message
	tf::quaternionTFToMsg(q,final.pose.orientation);

	used_imu = true;
	used_slam = true;

	current_pose = final;

	return final;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "poseCompleter");
	ros::NodeHandle n;
	
	// Subscribed topics
	ros::Subscriber imu_rec = n.subscribe<sensor_msgs::Imu>("/raw_imu",1000,imu_data);
	ros::Subscriber pose_rec = n.subscribe<geometry_msgs::PoseStamped>("/pose_stamped",1000,slam_data);

	//Published topics
	ros::Publisher poseCompleter_pub = n.advertise<geometry_msgs::PoseStamped>("/6d_pose", 1000);

	ros::Rate loop_rate(100);

	tf::TransformBroadcaster tf_sender;

	geometry_msgs::PoseStamped final_pose;
	tf::Stamped<tf::Pose> st;

	//We are ready when we get both imu and slam
	bool ready = false;

	while (ros::ok())
	{
		if (!used_imu and !used_slam)
			ready = true;

		//Only enter if we have new data
		if ((!used_imu or !used_slam) and ready == true)
		{
		//fuse the imu with slam
		final_pose = join_data(slam,imu);

		//publish transform and pose
		poseCompleter_pub.publish(final_pose);
		tf::poseStampedMsgToTF( final_pose,st);
		tf_sender.sendTransform(tf::StampedTransform(st, ros::Time::now(), "map", "6d_pose"));
		}

		ros::spinOnce();
	}
}
