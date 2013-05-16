
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <algorithm>

sensor_msgs::LaserScan inverted_scan;

bool publish_cloud=true;

bool ok=false;


void inversion(sensor_msgs::LaserScan scan)
{
	inverted_scan = scan;

	if (scan.angle_increment>0)
	{
		ok = true;
		return;
	}

	inverted_scan.angle_max = scan.angle_min;
	inverted_scan.angle_min = scan.angle_max;
	inverted_scan.angle_increment = -scan.angle_increment;
	std::reverse(inverted_scan.ranges.begin(),inverted_scan.ranges.end());
	ok = true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_inverter");
	 
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, inversion);
	
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("/scan2",1000);

	sensor_msgs::PointCloud2 pc2;
	while (ros::ok())
	{
		if (ok)
		{
			pub.publish(inverted_scan);

			if (publish_cloud==true)
			{



			}
		}

		ros::spinOnce();
	}

}
