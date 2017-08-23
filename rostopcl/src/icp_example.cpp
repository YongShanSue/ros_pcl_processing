#include <iostream> 
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
ros::Publisher pub;
ros::Publisher pointcloudXYZ;
ros::Publisher pointcloudicp;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_t0 (new pcl::PointCloud<pcl::PointXYZ>);
//cloud_icp_t0->points.resize(0);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_t1 (new pcl::PointCloud<pcl::PointXYZ>);
//cloud_icp_t1->points.resize(0);
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	 // Create a container for the data.
 	sensor_msgs::PointCloud2 output;
 	sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;

	// Do data processing here...
	output = *input;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
	if(cloud_icp_t0->points.size()==0){
		pcl::copyPointCloud(cloud,*cloud_icp_t0);
		pcl::copyPointCloud(cloud,*cloud_icp_t1);
		ROS_INFO("initial");
	}
	else{
		pcl::copyPointCloud(*cloud_icp_t1,*cloud_icp_t0);
		pcl::copyPointCloud(cloud,*cloud_icp_t1);		
		ROS_INFO("keep runnig");
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr registration_result (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_icp_t0);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_icp_t1);
	/*
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloud_icp_t0);
	icp.setInputTarget(cloud_icp_t1);
	icp.setMaxCorrespondenceDistance(1500);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setMaximumIterations(300);
	icp.align(*registration_result);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
		
	pcl::PointCloud<pcl::PointXYZ> icp_result;
	pcl::copyPointCloud(*registration_result,icp_result);
    //pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);
	*/
    //pub.publish (output);
	pointcloudXYZ.publish(*cloud_icp_t0);
	pointcloudicp.publish(*cloud_icp_t1);
	ROS_INFO("Success output");
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
	 cloud_icp_t0->points.resize(0);
	 cloud_icp_t1->points.resize(0);
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     pointcloudicp = nh.advertise<PointCloudXYZ> ("icp_result", 1);
     // Spin
	 ROS_INFO("Ros Start");
     ros::spin ();
  }
