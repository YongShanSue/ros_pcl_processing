#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <pcl/io/pcd_io.h>

using namespace message_filters;
using namespace cv;
using namespace std;

ros::Publisher hsv_mask;
ros::Publisher hsv_object_detection;
ros::Publisher pointcloud_now_publisher;
ros::Publisher pointcloud_model_publisher;
ros::Publisher path_publisher;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_model (new pcl::PointCloud<pcl::PointXYZ>);
nav_msgs::Path path;


struct  timeval    start;
struct  timeval   stop; 
double time_diff;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& image1)
{	
	gettimeofday(&stop,NULL);
	time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
	printf("wainting_time is %f\n",time_diff);
	gettimeofday(&start,NULL);
	
	 // Create a container for the data.
	//detect the object
	cv_bridge::CvImagePtr cv_ptr;
	Mat hsv ,hsv_detected_object ;

	gettimeofday(&start,NULL);
    cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);	
    Mat hsv_range_mask = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);	
    Mat hsv_range_mask_2 = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);
   
	cvtColor(cv_ptr->image,hsv,CV_BGR2HSV);
	inRange(hsv,Scalar(170,250,80) , Scalar(180,255,255), hsv_range_mask);		//red range1
	inRange(hsv,Scalar(0,250,80) , Scalar(1,255,255), hsv_range_mask_2);		//red range2
	bitwise_or(hsv_range_mask,hsv_range_mask_2, hsv_range_mask_2);

	for(int row=1;row<cv_ptr->image.rows;row++){
		for(int column=1;column<cv_ptr->image.cols;column++){
			cv_ptr->image.at<Vec3b>(row,column)[0] *=(hsv_range_mask_2.at<uchar>(row,column)/255.0);
			cv_ptr->image.at<Vec3b>(row,column)[1] *=(hsv_range_mask_2.at<uchar>(row,column)/255.0);
			cv_ptr->image.at<Vec3b>(row,column)[2] *=(hsv_range_mask_2.at<uchar>(row,column)/255.0);
		}
	}
	

	//cv_ptr->image.copyTo(test,hsv_range_mask );

	hsv_object_detection.publish(cv_ptr->toImageMsg());

	cv_bridge::CvImagePtr cv_ptr2;
 	cv_ptr2 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);	
	cv_ptr2->header   = cv_ptr->header; // Same timestamp and tf frame as input image
	cv_ptr2->encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	cv_ptr2->image    = hsv_range_mask; // Your cv::Mat
	hsv_mask.publish(cv_ptr2->toImageMsg());
 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now (new pcl::PointCloud<pcl::PointXYZ>);
 	gettimeofday(&stop,NULL);
	time_diff = (stop.tv_sec-start.tv_sec)+ stop.tv_usec-start.tv_usec;
	printf("generate_hsv_mask1_time is %ld\n",time_diff);
    
	gettimeofday(&start,NULL);	
 
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
	pcl::copyPointCloud(cloud,*cloud_now);
	//hsv filtering
	for(int row=0;row<cv_ptr->image.rows;row++){
		for(int column=0;column<cv_ptr->image.cols;column++){
			if(hsv_range_mask.at<uchar>(row+1,column+1)<127){
				cloud_now->points[row*cv_ptr->image.cols+column].x = std::numeric_limits<float>::quiet_NaN();
				cloud_now->points[row*cv_ptr->image.cols+column].y = std::numeric_limits<float>::quiet_NaN();
				cloud_now->points[row*cv_ptr->image.cols+column].z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}

	gettimeofday(&stop,NULL);
	time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
	printf("pointcloud_hsv_filtering_time is %f\n",time_diff);


	//gettimeofday(&start,NULL);
	std::vector<int> temp; 
	printf("Original point Size: %d %d %d\n",cloud_now->points.size(),cloud_now->width,cloud_now->height);	
	pcl::removeNaNFromPointCloud(*cloud_now, *cloud_now, temp); 
	printf("Filtered point Size: %d %d %d\n",cloud_now->points.size(),cloud_now->width,cloud_now->height);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr registration_result (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_now);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_icp_model);
	
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloud_now);
	icp.setInputTarget(cloud_icp_model);
	icp.setMaxCorrespondenceDistance(1500);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setMaximumIterations(300);
	icp.align(*cloud_icp_final);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	Eigen::Transform<float, 3, Eigen::Affine> tROTA(transformation);

	float x, y, z, roll, pitch, yaw;
	pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);	
	
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time();
	printf("%lf %lf %lf \n",x,y,z);
	p.pose.position.x = x;
 	p.pose.position.y = y;
	p.pose.position.z = z; // 1 meter
	path.poses.push_back(p);
	

	path_publisher.publish(path);

	gettimeofday(&stop,NULL);
	time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
	printf("ICP_time is %f\n",time_diff);

    printf("x: %lf, y: %lf, z: %lf, roll: %lf, pitch: %lf, yaw: %lf\n",x, y, z, roll, pitch, yaw);
    FILE *fPtr;
    fPtr = fopen("transformation.txt", "a");
    if (!fPtr) {
        printf("error\n");
        exit(1);
    }    
    fprintf(fPtr, "x: %lf, y: %lf, z: %lf, roll: %lf, pitch: %lf, yaw: %lf\n",x, y, z, roll, pitch, yaw);
    
    fclose(fPtr);
	std::cout << transformation << std::endl;
	pointcloud_now_publisher.publish(*cloud_now);
	pointcloud_model_publisher.publish(*cloud_icp_model);
	ROS_INFO("Success output");
	
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
	path.header.frame_id="/map";
	 pcl::io::loadPCDFile<pcl::PointXYZ> ("1.pcd", *cloud_icp_model);
	 printf("Model Size: %d %d %d\n",cloud_icp_model->points.size(),cloud_icp_model->width,cloud_icp_model->height);
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     hsv_object_detection = nh.advertise<sensor_msgs::Image> ("hsv_object_detection", 1); 
     hsv_mask = nh.advertise<sensor_msgs::Image> ("hsv_mask", 1); 
     pointcloud_now_publisher= nh.advertise<PointCloudXYZ> ("cloud_now", 1);
     pointcloud_model_publisher = nh.advertise<PointCloudXYZ> ("cloud_icp_model", 1);
     path_publisher = nh.advertise<nav_msgs::Path> ("cloud_path", 1);
    //pointcloudicp = nh.advertise<PointCloudXYZ> ("icp_result", 1);
     ///Synchonizer
     message_filters::Subscriber<sensor_msgs::PointCloud2> depth_point(nh,"/camera/depth/points", 10);
     message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 10);
  
     typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
     // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_point, image_sub);
     sync.registerCallback(boost::bind(&cloud_cb, _1, _2));
  



     // Spin
	 ROS_INFO("Ros Start");
     ros::spin ();
  }
