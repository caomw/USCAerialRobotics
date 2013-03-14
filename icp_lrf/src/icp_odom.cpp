#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <vector>
#include <iostream>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#ifndef PI
#define PI 3.141592
#endif

#define WAYPOINT_DIST_THRESH 0.1
#define WAYPOINT_ANGLE_THRESH 20*PI/180
#define ICP_MAX_ITERATIONS 50
#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.05


using namespace std;
using namespace Eigen;

struct float3{
    float x; float y; float z;
};

struct scan
{
    vector<float3> ranges;
    Matrix4f transform;
    void copyFrom(scan* src) {
	this->ranges.clear();
	for( int i=0; i<src->ranges.size(); i++)
	{
	    this->ranges.push_back(src->ranges[i]);
	}
	this->transform = src-> transform;
    }
    void resetTransform()
    {
	this->transform << 1, 0, 0, 0,
                           0, 1, 0, 0,
	                   0, 0, 1, 0,
	                   0, 0, 0, 1;
    }
	    
};

void updateTransform(scan& base, 
		     scan& target);

int argmin(vector<float> v)
{
    float x = v[0];
    int index = 0;
    for( int i=0; i<v.size(); i++)
    {
	index = v[i]<x?i:index;
	x =     v[i]<x?v[i]:x;
    }
    return index;
}

template<class T>
int sign(T x)
{
    return x>=0?1:-1;
}

float angleDist(float theta, float phi)
{
    vector<float> v;
    vector<float> w;

    v.push_back(theta-phi       );
    v.push_back(theta-phi - 2*PI);
    v.push_back(theta-phi + 2*PI);

    w.push_back(abs(v[0]));
    w.push_back(abs(v[1]));
    w.push_back(abs(v[2]));

    for( int i=0; i<w.size(); i++)
    {
	cout << w[i] << endl;
    }

    cout << argmin(w) << endl;

    return v[argmin(w)];
}

	


class line_finder {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_scan;
    ros::Publisher pub_cloud;
    ros::Publisher pub_lines;
    scan current_scan;
    scan base_scan;
    vector<scan*> base_history;
    bool first_execution;

    line_finder(ros::NodeHandle& _nh): nh(_nh) {
	pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/world", 1);
	sub_scan = nh.subscribe("/scan", 1, &line_finder::lrf_callback, this);
	first_execution = true;
    }

    ~line_finder() { 
    }
	
    void lrf_callback(const sensor_msgs::LaserScan::ConstPtr& msg_lrf) {
	ros::Subscriber sub_lines;


	int tot_scans = msg_lrf->ranges.size();
	float min_range = msg_lrf->range_min;
	float max_range = msg_lrf->range_max;
	double min_angle = msg_lrf->angle_min;
	double max_angle = msg_lrf->angle_max;
	double angle_increment = msg_lrf->angle_increment;

	current_scan.ranges.clear();
	for(int i = 0; i < msg_lrf->ranges.size(); i++) {
	    if(msg_lrf->ranges.size() != 0)
	    {
		float3 temp;
		temp.x = msg_lrf->ranges[i] * cos(min_angle + i*angle_increment);
		temp.y = msg_lrf->ranges[i] * sin(min_angle + i*angle_increment);
		temp.z = 0;
		current_scan.ranges.push_back(temp);
	    }
	}

	if(first_execution == true)
	{
	    for( int i=0; i<current_scan.ranges.size(); i++)
	    {
		base_scan.ranges.push_back(current_scan.ranges[i]);
	    }
	    first_execution = false;
	    base_scan.resetTransform();
	    current_scan.resetTransform();
	} 
	else
	{
	    

	    std::cout << "base_size:" << base_scan.ranges.size() << std::endl;
	    std::cout << "target_size:" << current_scan.ranges.size() << std::endl;

	    updateTransform(base_scan, current_scan);
	    cout << "base_scan: " << endl << base_scan.transform << endl << endl;
	    cout << "current_scan: " << endl << current_scan.transform << endl << endl;

	    // float angle_diff = angleDist(atan2(base_scan.transform(1,0)   , base_scan.transform(0, 0)),
	    // 				 atan2(current_scan.transform(1,0), current_scan.transform(0, 0)));
	    float angle_diff = angleDist(atan2(current_scan.transform(1,0), current_scan.transform(0, 0)), 0);
	    float base_angle = 		 atan2(base_scan.transform(1,0), base_scan.transform(0, 0)) + angle_diff;

	    cout << "angle_diff = " << angle_diff*180/PI << endl;
	    cout << "base_angle = " << base_angle*180/PI << endl;
	    

	    if( (pow(pow(current_scan.transform(0,3),2) +
	    	     pow(current_scan.transform(1,3),2) ,0.5) >
	    	 WAYPOINT_DIST_THRESH) || 
	    	(abs(angle_diff) > WAYPOINT_ANGLE_THRESH) )
	    {
	    	cout << endl << endl << "New base scan detected !!!!!!!!!!!!!!!!!!!!" << endl << endl << endl;

		base_scan.ranges.clear();
	    	for(int i=0; i<current_scan.ranges.size(); i++)
	    	{
	    	    base_scan.ranges.push_back(current_scan.ranges[i]);
	    	}
	    	base_scan.transform = current_scan.transform * base_scan.transform;
	    	current_scan.resetTransform();
	
	     }


    	}


    }
};


int main (int argc, char** argv) {
    ros::init(argc, argv, "line_finder");
    ros::NodeHandle nh;
	
    line_finder lf(nh);


    ros::Rate loop_rate(10);
    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}


void updateTransform(scan& base, 
		  scan& target)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);



    // Fill in the CloudIn data
    cloud_in->width    = base.ranges.size();
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
	Vector4f ego_base;
	Vector4f allo_base;
	ego_base << base.ranges[i].x, base.ranges[i].y, base.ranges[i].z, 1;
	allo_base = target.transform * ego_base;
	
	cloud_in->points[i].x = allo_base(0);
	cloud_in->points[i].y = allo_base(1);
	cloud_in->points[i].z = allo_base(2);
    }

    cloud_out->width    = target.ranges.size();
    cloud_out->height   = 1;
    cloud_out->is_dense = false;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);

    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
	cloud_out->points[i].x = target.ranges[i].x;
	cloud_out->points[i].y = target.ranges[i].y;
	cloud_out->points[i].z = target.ranges[i].z;
    }


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setMaximumIterations(ICP_MAX_ITERATIONS);
    icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl; 
    //std::cout << icp.getFinalTransformation() << std::endl;
    target.transform *= icp.getFinalTransformation();
    //std::cout << target.transform * base.transform << std::endl;
}
