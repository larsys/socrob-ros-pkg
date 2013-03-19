#include <ros/ros.h>
#include <pcl/point_cloud.h>
//#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <limits>
#include <math.h>

#include <kinect_driver/pointcloud_compress_msg.h>

ros::Publisher pub_pointcloud_decompress;

void
pointcloud_decompressor(const kinect_driver::pointcloud_compress_msg::ConstPtr &pointcloud_compress){

	pcl::PointCloud<pcl::PointXYZRGB> pointcloud_decompress;
	pointcloud_decompress.header.frame_id=pointcloud_compress->header.frame_id;
	pointcloud_decompress.width=pointcloud_compress->width;
	pointcloud_decompress.height=pointcloud_compress->height;
	pointcloud_decompress.is_dense=false;
	pointcloud_decompress.points.resize(pointcloud_decompress.width*pointcloud_decompress.height);

	int u,v,index=0;
	for(unsigned int bit=0;bit<pointcloud_decompress.width*pointcloud_decompress.height;bit++) {

			u=floor(bit/pointcloud_decompress.height);
			v=bit%pointcloud_decompress.height;

			if(!((uint64_t )(pointcloud_compress->nan_points[floor(bit/64)]) & (uint64_t )(pow(2,bit%64)))) {
				
				pointcloud_decompress(u,v).x=(*pointcloud_compress).x[index];
				pointcloud_decompress(u,v).y=(*pointcloud_compress).y[index];
				pointcloud_decompress(u,v).z=(*pointcloud_compress).z[index];
				
				pointcloud_decompress(u,v).r=(*pointcloud_compress).r[index];
				pointcloud_decompress(u,v).g=(*pointcloud_compress).g[index];
				pointcloud_decompress(u,v).b=(*pointcloud_compress).b[index];
				
				index++;
			}else{
			
				pointcloud_decompress(u,v).x=std::numeric_limits<float>::quiet_NaN();
				pointcloud_decompress(u,v).y=std::numeric_limits<float>::quiet_NaN();
				pointcloud_decompress(u,v).z=std::numeric_limits<float>::quiet_NaN();
				
				pointcloud_decompress(u,v).r=0;
				pointcloud_decompress(u,v).g=0;
				pointcloud_decompress(u,v).b=0;
			}
	}
	
	//ROS_INFO(" Numero de pontos activos %i",index);
	
	pointcloud_decompress.header.stamp=pointcloud_compress->header.stamp;
	//pointcloud_decompress.header.stamp=ros::Time::now();
	pub_pointcloud_decompress.publish(pointcloud_decompress);	  
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_decompressor_node");
  ros::NodeHandle node_handler;
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node_handler.subscribe< kinect_driver::pointcloud_compress_msg > ("pointcloud_compress", 1, pointcloud_decompressor);
  
  // Create a ROS publisher for the output point cloud
  pub_pointcloud_decompress = node_handler.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("pointcloud_decompress", 1);

  // Spin
  ros::spin ();
}
