#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <vector>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>




#include <art_lrf/Lines.h>
#include <Eigen/Eigen>

#ifndef PI
#define PI 3.141592
#endif

using namespace std;
using namespace Eigen;

bool first_execution = true;

struct float3{
    float x;
    float y;
    float z;
};

void getTransform(vector<float3> base,
		  vector<float3> target);

class IMU_odom
{

public:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_arduino;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::Vector3Stamped> Policy_sync_subs;
    message_filters::Synchronizer<Policy_sync_subs> sync_subs;
    ros::Publisher pub_cloud;
    ros::Publisher pub_lines;

    vector<float3> current_scan;
    vector<float3> base_scan;



    IMU_odom(ros::NodeHandle& _nh): nh(_nh), sub_scan(nh, "/scan", 10), sub_arduino(nh, "/arduino/yaw", 10), 
				       sync_subs(Policy_sync_subs(10), sub_scan, sub_arduino) 
	{
	    //pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/world", 10);
	    //pub_lines = nh.advertise<art_lrf::Lines>("/lrfLines", 10);
	    sync_subs.registerCallback(boost::bind(&IMU_odom::sync_subs_callback, this, _1, _2));
	}


    ~IMU_odom() 
	{ 
	    //old_lines.reset();
	    //current_lines.reset();
	}

    void sync_subs_callback(const sensor_msgs::LaserScan::ConstPtr& msg_lrf, const geometry_msgs::Vector3Stamped::ConstPtr& msg_arduino) 
	{
	    ros::Subscriber sub_lines;
	    current_scan.clear();

	    int tot_scans = msg_lrf->ranges.size();
	    float min_range = msg_lrf->range_min;
	    float max_range = msg_lrf->range_max;
	    double min_angle = msg_lrf->angle_min;
	    double max_angle = msg_lrf->angle_max;
	    double angle_increment = msg_lrf->angle_increment;

	    for(int i = 0; i < msg_lrf->ranges.size(); i++) {
		float3 temp;
		temp.x = msg_lrf->ranges[i] * cos(min_angle + i*angle_increment);
		temp.y = msg_lrf->ranges[i] * sin(min_angle + i*angle_increment);
		temp.z = 0;
		current_scan.push_back(temp);
	    }

	    if(first_execution == true)
	    {
		for( int i=0; i<current_scan.size(); i++)
		{
		    base_scan.push_back(current_scan[i]);
		}
		first_execution = false;
	    }
        
	    getTransform(base_scan, current_scan);
	
	    //pub_lines.publish();
	}
};


// class line_finder {
// public:
//     ros::NodeHandle nh;
//     ros::Subscriber sub_scan;
//     ros::Publisher pub_cloud;
//     ros::Publisher pub_lines;
//     vector<float3> current_scan;
//     vector<float3> base_scan;

//     line_finder(ros::NodeHandle& _nh): nh(_nh) {
// 	pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/world", 1);
// 	sub_scan = nh.subscribe("/scan", 1, &line_finder::lrf_callback, this);
//     }

//     ~line_finder() { 
//     }
	
//     void lrf_callback(const sensor_msgs::LaserScan::ConstPtr& msg_lrf) {
// 	ros::Subscriber sub_lines;
// 	current_scan.clear();

// 	int tot_scans = msg_lrf->ranges.size();
// 	float min_range = msg_lrf->range_min;
// 	float max_range = msg_lrf->range_max;
// 	double min_angle = msg_lrf->angle_min;
// 	double max_angle = msg_lrf->angle_max;
// 	double angle_increment = msg_lrf->angle_increment;

// 	for(int i = 0; i < msg_lrf->ranges.size(); i++) {
// 	    float3 temp;
// 	    temp.x = msg_lrf->ranges[i] * cos(min_angle + i*angle_increment);
// 	    temp.y = msg_lrf->ranges[i] * sin(min_angle + i*angle_increment);
// 	    temp.z = 0;
// 	    current_scan.push_back(temp);
// 	}

// 	if(first_execution == true)
// 	{
// 	    for( int i=0; i<current_scan.size(); i++)
// 	    {
// 		base_scan.push_back(current_scan[i]);
// 	    }
// 	    first_execution = false;
// 	}
        
// 	getTransform(base_scan, current_scan);
	
// 	//pub_lines.publish();
//     }
// };


int main (int argc, char** argv) {
    ros::init(argc, argv, "IMU_odom");
    ros::NodeHandle nh;
	
    IMU_odom lf(nh);


    ros::Rate loop_rate(10);
    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}


void getTransform(vector<float3> base, 
		  vector<float3> target)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the CloudIn data
    cloud_in->width    = base.size();
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
	cloud_in->points[i].x = base[i].x;
	cloud_in->points[i].y = base[i].y;
	cloud_in->points[i].z = base[i].z;
    }

    cloud_out->width    = target.size();
    cloud_out->height   = 1;
    cloud_out->is_dense = false;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);

    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
	cloud_out->points[i].x = target[i].x;
	cloud_out->points[i].y = target[i].y;
	cloud_out->points[i].z = target[i].z;
    }

    std::cout << "base_size:" << cloud_in->points.size() << std::endl;
    std::cout << "target_size:" << cloud_out->points.size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
}
