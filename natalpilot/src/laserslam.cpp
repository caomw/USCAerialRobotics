#include "ros/ros.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl/registration/icp.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include <cmath>
#include <iostream>

typedef pcl::PointXYZ PointT;

ros::Publisher oldpub;
ros::Publisher newpub;
ros::Subscriber sub;

pcl::IterativeClosestPoint<PointT,PointT> icp;

pcl::PointCloud<PointT> oldPointCloud;
pcl::PointCloud<PointT> newPointCloud;
pcl::PointCloud<PointT> midPointCloud;
sensor_msgs::PointCloud tempPointCloud;
sensor_msgs::PointCloud2 tempPointCloud2;

laser_geometry::LaserProjection lp;
Eigen3::Matrix4f tf_result;
int runcount = 0;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	lp.projectLaser(*msg,tempPointCloud);
	sensor_msgs::convertPointCloudToPointCloud2(tempPointCloud,tempPointCloud2);
	pcl::fromROSMsg(tempPointCloud2,newPointCloud);	

	if(runcount == 10*10)
	{
		cout << endl << "got old!" << endl;
		oldPointCloud = newPointCloud;
		pcl::toROSMsg(oldPointCloud,tempPointCloud2);
		tempPointCloud.header.frame_id = "/oldpoints";
		oldpub.publish(tempPointCloud2);
	}
	else if(runcount == 128)
	{
		cout << endl << "got new!" << endl;
		
		if(oldPointCloud.points.size() > newPointCloud.points.size())
			do{oldPointCloud.points.pop_back();}while(oldPointCloud.points.size() > newPointCloud.points.size());
		else if(oldPointCloud.points.size() < newPointCloud.points.size())
			do{newPointCloud.points.pop_back();}while(oldPointCloud.points.size() < newPointCloud.points.size());

		for(int i=0;i<newPointCloud.points.size();i++){
			newPointCloud.points[i].x += -5;
			newPointCloud.points[i].y += -5;
		}

		icp.setInputCloud(boost::make_shared<const pcl::PointCloud<PointT> > (newPointCloud));
		icp.setInputTarget(boost::make_shared<const pcl::PointCloud<PointT> > (oldPointCloud));
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(1e-8);
		icp.setMaxCorrespondenceDistance(7.2);
		icp.align (midPointCloud);
		tf_result = icp.getFinalTransformation();

		std::cout << endl << endl << tf_result << endl << endl;

		//---apply transformation matrix

		for(int i=0;i<newPointCloud.points.size();i++){
			Eigen3::Vector4f v_point(newPointCloud.points[i].x,newPointCloud.points[i].y,newPointCloud.points[i].z,1.0);
			v_point = tf_result * v_point;
			newPointCloud.points[i].x = v_point(0,0);
			newPointCloud.points[i].y = v_point(1,0);
			newPointCloud.points[i].z = v_point(2,0);
		}

		//---end of applying

		pcl::toROSMsg(midPointCloud,tempPointCloud2);
		tempPointCloud.header.frame_id = "/newpoints";
		newpub.publish(tempPointCloud2);
	}

	static tf::TransformBroadcaster broadcaster;
	tf::Transform pub_tf(tf::createQuaternionFromRPY(0.0,0.0,0.0),tf::Vector3(0.0, 0.0, 0.0));
	broadcaster.sendTransform(tf::StampedTransform(pub_tf,ros::Time::now(),"/world", "/laser"));
	broadcaster.sendTransform(tf::StampedTransform(pub_tf,ros::Time::now(),"/world", "/oldpoints"));
	broadcaster.sendTransform(tf::StampedTransform(pub_tf,ros::Time::now(),"/world", "/newpoints"));

	runcount++;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserslam");
	ros::NodeHandle n;
	oldpub = n.advertise<sensor_msgs::PointCloud2>("/oldpoints", 1000);
	newpub = n.advertise<sensor_msgs::PointCloud2>("/newpoints", 1000);
	sub = n.subscribe("/scan", 1000, scan_callback);

	ros::spin();
	return 0;
}

