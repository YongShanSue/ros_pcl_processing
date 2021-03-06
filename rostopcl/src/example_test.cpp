#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
ros::Publisher pub;
ros::Publisher pointcloudXYZ;
ros::Publisher pointcloud2_publisher;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
struct  timeval    start;
struct  timeval   stop; 
double time_diff;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	/*
     // Create a container for the data.
 	sensor_msgs::PointCloud2 output;
 	sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;

	// Do data processing here...
	output = *input;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
    pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);

    pub.publish (output);
	pointcloudXYZ.publish(cloud);
	pointcloud2_publisher.publish(pcl_to_ros_pointcloud2);
	ROS_INFO("Success output");
    */
    gettimeofday(&stop,NULL);

    time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
    printf("wainting_time is %f\n",time_diff);
    gettimeofday(&start,NULL);
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
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2> ("pcltoros_pointcloud2", 1);
     // Spin
     ros::spin ();
  }
