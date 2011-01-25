#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "pcl/registration/icp_nl.h"
#include "pcl/common/transformation_from_correspondences.h"
#include <cmath>

typedef pcl::PointXYZ PointT;

pcl::IterativeClosestPointNonLinear<PointT,PointT> icp;
pcl::PointCloud<PointT> oldPointCloud;
pcl::PointCloud<PointT> newPointCloud;
Eigen3::Matrix4f tf_result;
bool firstrun = true;

struct TRANSFORM{
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} result;

void sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	oldPointCloud = newPointCloud;
	pcl::fromROSMsg(*msg,newPointCloud);
	if(firstrun){
		firstrun = false;
	}else{
		icp.estimateRigidTransformationLM(oldPointCloud,newPointCloud,tf_result);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "natalpilot");
	ros::NodeHandle n;

	icp.setInputCloud(boost::make_shared< pcl::PointCloud<PointT> > (oldPointCloud));
	icp.setInputTarget(boost::make_shared< pcl::PointCloud<PointT> > (newPointCloud));

	ros::Subscriber sub = n.subscribe("camera/points2", 1000, sub_callback);

	ros::spin();
	return 0;
}

