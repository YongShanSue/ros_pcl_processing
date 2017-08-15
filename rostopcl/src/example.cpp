#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
ros::Publisher pub;
ros::Publisher pointcloudXYZ;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	 // Create a container for the data.
 	sensor_msgs::PointCloud2 output;
	// Do data processing here...
	output = *input;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
     	// Publish the data.
     	pub.publish (output);
	pointcloudXYZ.publish(cloud);
	ROS_INFO("Success output");
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("pointcloudXYZ", 1);
     // Spin
     ros::spin ();
  }
