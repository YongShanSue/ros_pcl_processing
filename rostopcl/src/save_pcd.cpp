#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <sstream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h> //pcl-1.7/

//Synchronize
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//opencv, cv bridge
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>

using namespace message_filters;
using namespace cv;
using namespace std;
ros::Publisher hsv_mask;
ros::Publisher hsv_object_detection;
ros::Publisher pointcloud_model;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_model (new pcl::PointCloud<pcl::PointXYZ>);


int num=1;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;



void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& image1)
{	

	 // Create a container for the data.
	//detect the object
	cv_bridge::CvImagePtr cv_ptr;
 
	Mat hsv ,hsv_detected_object ;

    cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);	
    Mat hsv_range_mask = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);	
    Mat hsv_range_mask_2 = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);
    
	cvtColor(cv_ptr->image,hsv,CV_BGR2HSV);
	inRange(hsv,Scalar(170,250,80) , Scalar(180,256,256), hsv_range_mask);		//red range1
	inRange(hsv,Scalar(-1,250,80) , Scalar(1,256,256), hsv_range_mask_2);		//red range2
	bitwise_or(hsv_range_mask,hsv_range_mask_2, hsv_range_mask_2);

	for(int row=1;row<cv_ptr->image.rows;row++){
		for(int column=1;column<cv_ptr->image.cols;column++){
			cv_ptr->image.at<Vec3b>(row,column)[0] *=(hsv_range_mask_2.at<uchar>(row,column)/255.0);
			cv_ptr->image.at<Vec3b>(row,column)[1] *=(hsv_range_mask_2.at<uchar>(row,column)/255.0);
			cv_ptr->image.at<Vec3b>(row,column)[2] *=(hsv_range_mask_2.at<uchar>(row,column)/255.0);
		}
	}
	hsv_object_detection.publish(cv_ptr->toImageMsg());


	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
	pcl::copyPointCloud(cloud,*cloud_icp_model);
	//hsv filtering
	for(int row=0;row<cv_ptr->image.rows;row++){
		for(int column=0;column<cv_ptr->image.cols;column++){
			if(hsv_range_mask.at<uchar>(row+1,column+1)<127){
				cloud_icp_model->points[row*cv_ptr->image.cols+column].x = std::numeric_limits<float>::quiet_NaN();
				cloud_icp_model->points[row*cv_ptr->image.cols+column].y = std::numeric_limits<float>::quiet_NaN();
				cloud_icp_model->points[row*cv_ptr->image.cols+column].z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
	


	std::vector<int> temp; 
	pcl::removeNaNFromPointCloud(*cloud_icp_model, *cloud_icp_model, temp); 
	pointcloud_model.publish(*cloud_icp_model);
	int check_save=0;
	scanf("%d",&check_save);
	if(check_save>=1){
		char str[200]; 
		sprintf(str,"%d.pcd",num); //~/duckietown/catkin_ws/src/ros_pcl_processing/rostopcl/src/model/
		printf("%s\n",str);
		pcl::io::savePCDFileASCII (str, *cloud_icp_model);
		ROS_INFO("Success output");
		num++;
	}
	
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     hsv_object_detection = nh.advertise<sensor_msgs::Image> ("hsv_object_detection", 1); 
     pointcloud_model= nh.advertise<PointCloudXYZ> ("model_visualzation", 1);
     ///Synchonizer
     message_filters::Subscriber<sensor_msgs::PointCloud2> depth_point(nh,"/camera/depth/points", 1);
     message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);
  
     typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
     // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_point, image_sub);
     sync.registerCallback(boost::bind(&cloud_cb, _1, _2));
  

     // Spin
	 ROS_INFO("Ros Start");
     ros::spin ();
  }
