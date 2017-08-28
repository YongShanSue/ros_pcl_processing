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
ros::Publisher pointcloudXYZ;
ros::Publisher pointcloudicp;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_t0 (new pcl::PointCloud<pcl::PointXYZ>);
//cloud_icp_t0->points.resize(0);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_t1 (new pcl::PointCloud<pcl::PointXYZ>);
//cloud_icp_t1->points.resize(0);
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
	/*
	 // Create a container for the data.
	//detect the object
	cv_bridge::CvImagePtr cv_ptr;
 
	Mat hsv ,hsv_detected_object ;


	gettimeofday(&start,NULL);
    cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);	
    Mat hsv_range_mask = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);	
    Mat hsv_range_mask_2 = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);
    bitwise_or(hsv_range_mask,hsv_range_mask_2, hsv_range_mask_2);
	cvtColor(cv_ptr->image,hsv,CV_BGR2HSV);
	inRange(hsv,Scalar(170,250,130) , Scalar(180,255,255), hsv_range_mask);		//red range1
	inRange(hsv,Scalar(0,250,130) , Scalar(1,255,255), hsv_range_mask_2);		//red range2


	for(int row=1;row<cv_ptr->image.rows;row++){
		for(int column=1;column<cv_ptr->image.cols;column++){
			cv_ptr->image.at<Vec3b>(row,column)[0] *=(hsv_range_mask.at<uchar>(row,column)/255.0);
			cv_ptr->image.at<Vec3b>(row,column)[1] *=(hsv_range_mask.at<uchar>(row,column)/255.0);
			cv_ptr->image.at<Vec3b>(row,column)[2] *=(hsv_range_mask.at<uchar>(row,column)/255.0);
		}
	}
	

	//cv_ptr->image.copyTo(test,hsv_range_mask );

	hsv_object_detection.publish(cv_ptr->toImageMsg());
	gettimeofday(&stop,NULL);
	time_diff = (stop.tv_sec-start.tv_sec)+ stop.tv_usec-start.tv_usec;
	printf("generate_hsv_mask1_time is %ld\n",time_diff);
    
	gettimeofday(&start,NULL);
	cv_bridge::CvImagePtr cv_ptr2;
 	cv_ptr2 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);	
	cv_ptr2->header   = cv_ptr->header; // Same timestamp and tf frame as input image
	cv_ptr2->encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	cv_ptr2->image    = hsv_range_mask; // Your cv::Mat
	hsv_mask.publish(cv_ptr2->toImageMsg());
	gettimeofday(&stop,NULL);
	time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
	printf("generate_hsv_mask2_time is %f\n",time_diff);


	//free memory
	//hsv_range_mask.release();
	//hsv.release();
	//hsv_detected_object.release();



	gettimeofday(&start,NULL);
 	sensor_msgs::PointCloud2 output;
 	sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;

	// Do data processing here...
 	
 

	output = *input;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
	if(cloud_icp_t0->points.size()==0){
		pcl::copyPointCloud(cloud,*cloud_icp_t0);
		//hsv filtering
		for(int row=0;row<cv_ptr->image.rows;row++){
			for(int column=0;column<cv_ptr->image.cols;column++){
				if(hsv_range_mask.at<uchar>(row+1,column+1)<127){
					cloud_icp_t0->points[row*cv_ptr->image.cols+column].x = std::numeric_limits<float>::quiet_NaN();
					cloud_icp_t0->points[row*cv_ptr->image.cols+column].y = std::numeric_limits<float>::quiet_NaN();
					cloud_icp_t0->points[row*cv_ptr->image.cols+column].z = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}
		pcl::copyPointCloud(*cloud_icp_t0,*cloud_icp_t1);
		ROS_INFO("initial");
	}
	else{

		pcl::copyPointCloud(*cloud_icp_t1,*cloud_icp_t0);
		pcl::copyPointCloud(cloud,*cloud_icp_t1);	
		//hsv filtering
		for(int row=0;row<cv_ptr->image.rows;row++){
			for(int column=0;column<cv_ptr->image.cols;column++){
				if(hsv_range_mask.at<uchar>(row+1,column+1)<127){
					cloud_icp_t1->points[row*cv_ptr->image.cols+column].x = std::numeric_limits<float>::quiet_NaN();
					cloud_icp_t1->points[row*cv_ptr->image.cols+column].y = std::numeric_limits<float>::quiet_NaN();
					cloud_icp_t1->points[row*cv_ptr->image.cols+column].z = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}	
		//ROS_INFO("keep runnig");
	}
	gettimeofday(&stop,NULL);
	time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
	printf("pointcloud_hsv_filtering_time is %f\n",time_diff);


	gettimeofday(&start,NULL);
	std::vector<int> temp; 
	printf("Original t0 Size: %d %d %d\n",cloud_icp_t0->points.size(),cloud_icp_t0->width,cloud_icp_t0->height);
	printf("Original t1 Size: %d %d %d\n",cloud_icp_t1->points.size(),cloud_icp_t1->width,cloud_icp_t1->height);
	pcl::removeNaNFromPointCloud(*cloud_icp_t1, *cloud_icp_t1, temp); 
	pcl::removeNaNFromPointCloud(*cloud_icp_t0, *cloud_icp_t0, temp); 
	printf("Filtered t0 Size: %d %d %d\n",cloud_icp_t0->points.size(),cloud_icp_t0->width,cloud_icp_t0->height);
	printf("Filtered t1 Size: %d %d %d\n",cloud_icp_t1->points.size(),cloud_icp_t1->width,cloud_icp_t1->height);

	pcl::PointCloud<pcl::PointXYZ>::Ptr registration_result (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_icp_t0);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_icp_t1);
	
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
	Eigen::Transform<float, 3, Eigen::Affine> tROTA(transformation);
	float x, y, z, roll, pitch, yaw;
	pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);	


	pcl::PointCloud<pcl::PointXYZ> icp_result;
	pcl::copyPointCloud(*registration_result,icp_result);
	gettimeofday(&stop,NULL);
	time_diff =  (stop.tv_sec-start.tv_sec)+ (stop.tv_usec-start.tv_usec)/1000000.0;
	printf("ICP_time is %f\n",time_diff);

	printf("ICP point Size: %d %d %d\n",registration_result->points.size(),registration_result->width,registration_result->height);
    printf("x: %lf, y: %lf, z: %lf, roll: %lf, pitch: %lf, yaw: %lf\n",x, y, z, roll, pitch, yaw);
    FILE *fPtr;
    fPtr = fopen("transformation.txt", "a");
    if (!fPtr) {
        printf("error\n");
        exit(1);
    }    
    fprintf(fPtr, "x: %lf, y: %lf, z: %lf, roll: %lf, pitch: %lf, yaw: %lf\n",x, y, z, roll, pitch, yaw);
    
    fclose(fPtr);
    //pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);
	std::cout << transformation << std::endl;
    //pub.publish (output);
	pointcloudXYZ.publish(*cloud_icp_t0);
	pointcloudicp.publish(*registration_result);
	ROS_INFO("Success output");
	*/
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
	 cloud_icp_t0->points.resize(0);
	 cloud_icp_t1->points.resize(0);
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     hsv_object_detection = nh.advertise<sensor_msgs::Image> ("hsv_object_detection", 1); 
     hsv_mask = nh.advertise<sensor_msgs::Image> ("hsv_mask", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     pointcloudicp = nh.advertise<PointCloudXYZ> ("icp_result", 1);
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
