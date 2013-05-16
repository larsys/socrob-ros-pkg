#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <limits>
#include <boost/array.hpp>
#include <math.h>
#include <time.h>
#include <string>

#include <kinect_driver/pointcloud_compress_msg.h>

ros::Publisher pub_pointcloud_compress;

void 
pointcloud_compressor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointcloud){

	kinect_driver::pointcloud_compress_msg msg;
	msg.width=(uint16_t )(pointcloud->width);
	msg.height=(uint16_t )(pointcloud->height);
	msg.nan_points.resize(ceil(msg.width*msg.height/64));
	msg.header.frame_id=pointcloud->header.frame_id;
	//msg.nan_points.fill(msg.nan_points.begin(), 640*480, 0);
	int n_nan_points=0;
	
	
	unsigned int v,u,linear_index;
	for( v=0;v<pointcloud->height;v++) {
		for( u=0;u<pointcloud->width;u++) {
		
			linear_index=v+u*pointcloud->height;
			
			if(isnan((*pointcloud)(u,v).z)){
				//an uint64 array with size 4800 to perform 640*480=307200 bits.
				msg.nan_points[floor(linear_index/64)]=msg.nan_points[floor(linear_index/64)]+(uint64_t )(pow(2,linear_index%64));
				n_nan_points++;
			}		
		}
	}
	
	//ROS_INFO("*Numero de pixeis inactivos %i",n_nan_points);
	
	msg.x.resize(msg.width*msg.height-n_nan_points);
	msg.y.resize(msg.width*msg.height-n_nan_points);
	msg.z.resize(msg.width*msg.height-n_nan_points);
	msg.r.resize(msg.width*msg.height-n_nan_points);
	msg.g.resize(msg.width*msg.height-n_nan_points);
	msg.b.resize(msg.width*msg.height-n_nan_points);
	
	
	int index=0;
	for(int bit=0;bit<msg.width*msg.height;bit++) {
			
			
			if(!((uint64_t )(msg.nan_points[floor(bit/64)]) & (uint64_t )(pow(2,bit%64)))) {
				u=floor(bit/pointcloud->height);
				v=bit%pointcloud->height;
				
				msg.x[index]=(*pointcloud)(u,v).x;
				msg.y[index]=(*pointcloud)(u,v).y;
				msg.z[index]=(*pointcloud)(u,v).z;
				
				msg.r[index]=(*pointcloud)(u,v).r;
				msg.g[index]=(*pointcloud)(u,v).g;
				msg.b[index]=(*pointcloud)(u,v).b;
				
				index++;
			}
	}
	
	//ROS_INFO(" Numero de pontos activos %i",index);
	//ROS_INFO(" Numero de pontos totais %i",index+n_nan_points);
	
	msg.header.stamp=pointcloud->header.stamp;
	//msg.header.stamp=ros::Time::now();	
	pub_pointcloud_compress.publish(msg);	  

	//ROS_INFO("Uma compresao estimada de %f",((640*480-n_nan_points)*3*4+4800*8.0)/(640*480*3*4.0));
	
	
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_compressor_node");
  ros::NodeHandle node_handler;
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node_handler.subscribe< pcl::PointCloud<pcl::PointXYZRGB> > ("pointcloud", 1, pointcloud_compressor);
  
  // Create a ROS publisher for the output point cloud
  pub_pointcloud_compress = node_handler.advertise< kinect_driver::pointcloud_compress_msg > ("pointcloud_compress", 1);

  // Spin
  ros::spin ();
}
