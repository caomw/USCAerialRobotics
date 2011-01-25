#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

void sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg){

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*msg,cloud);
	pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	ROS_INFO ("Saved %d data points to test_pcd.pcd.", (int)cloud.points.size ());
}

int main (int argc, char** argv){
	ros::init(argc, argv, "pccapture");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/kinect/points2", 10, sub_callback);
	getchar();
	ros::spin();
  	return (0);
}

