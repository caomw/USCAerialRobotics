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

<<<<<<< HEAD
using namespace std;
using namespace Eigen;

bool first_execution = true;


class float3{
public:
    float x; float y; float z;
    float3(float a, float b, float c){
	x = a; y = b; z = c;
    }
    float3(){
	x = 0; y = 0; z = 0;
    }
};

void getTransform(vector<float3> base,
		  vector<float3> target);


class line_finder {

=======
#define WAYPOINT_DIST_THRESH 0.3
#define WAYPOINT_ANGLE_THRESH 30*PI/180

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

};

void updateTransform(scan& base, 
		     scan& target);

float getMin(vector<float> v)
{
    float x = v[0];
    for( int i=0; i<v.size(); i++)
    {
	x = x<v[i]?x:v[i];
    }
    return x;
}

float angleDist(float theta, float phi)
{
    vector<float> v;
    v.push_back(abs(theta-phi        ));
    v.push_back(abs(theta-phi - 2*PI));
    v.push_back(abs(theta-phi + 2*PI));

    return getMin(v);
}

	


class line_finder {
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_scan;
    ros::Publisher pub_cloud;
    ros::Publisher pub_lines;
<<<<<<< HEAD
    vector<float3> current_scan;
    vector<float3> base_scan;

    line_finder(ros::NodeHandle& _nh): nh(_nh) {
	pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/cloud", 1);
	sub_scan = nh.subscribe("/scan", 1, &line_finder::lrf_callback, this);
=======
    scan current_scan;
    scan base_scan;
    vector<scan*> base_history;
    bool first_execution;

    line_finder(ros::NodeHandle& _nh): nh(_nh) {
	pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/world", 1);
	sub_scan = nh.subscribe("/scan", 1, &line_finder::lrf_callback, this);
	first_execution = true;
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
    }

    ~line_finder() { 
    }
	
    void lrf_callback(const sensor_msgs::LaserScan::ConstPtr& msg_lrf) {
	ros::Subscriber sub_lines;
<<<<<<< HEAD
	current_scan.clear();
=======

>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92

	int tot_scans = msg_lrf->ranges.size();
	float min_range = msg_lrf->range_min;
	float max_range = msg_lrf->range_max;
	double min_angle = msg_lrf->angle_min;
	double max_angle = msg_lrf->angle_max;
	double angle_increment = msg_lrf->angle_increment;

<<<<<<< HEAD
	for(int i = 0; i < msg_lrf->ranges.size(); i++) {
	    float3 temp;
	    temp.x = msg_lrf->ranges[i] * cos(min_angle + i*angle_increment);
	    temp.y = msg_lrf->ranges[i] * sin(min_angle + i*angle_increment);
	    temp.z = 0;
	    current_scan.push_back(temp);
=======

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
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
	}

	if(first_execution == true)
	{
<<<<<<< HEAD
	    for( int i=0; i<current_scan.size(); i++)
	    {
		base_scan.push_back(current_scan[i]);
	    }
	    first_execution = false;
	}
        
	getTransform(base_scan, current_scan);
	
	//pub_lines.publish();
=======
	    for( int i=0; i<current_scan.ranges.size(); i++)
	    {
		base_scan.ranges.push_back(current_scan.ranges[i]);
	    }
	    first_execution = false;
	    base_scan.transform << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	    current_scan.transform << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	} 
	else
	{
	    
	    updateTransform(base_scan, current_scan);

	    // float angle_diff = angleDist(atan2(base_scan.transform(1,0)   , base_scan.transform(0, 0)),
	    // 				 atan2(current_scan.transform(1,0), current_scan.transform(0, 0)));
	    float angle_diff = angleDist(0,
	    				 atan2(current_scan.transform(1,0), current_scan.transform(0, 0)));

	    cout << "angle_diff = " << angle_diff*180/PI << endl;

	    // if( (pow(pow(base_scan.transform(0,3) - current_scan.transform(0,3),2) +
	    // 	     pow(base_scan.transform(1,3) - current_scan.transform(1,3),2) ,0.5) >
	    // 	 WAYPOINT_DIST_THRESH) || 
	    // 	(angle_diff > WAYPOINT_ANGLE_THRESH) )
	    // {
	    // 	cout << endl << endl << "New base scan detected !!!!!!!!!!!!!!!!!!!!" << endl << endl << endl;

	    // 	scan new_base;
	    // 	for(int i=0; i<current_scan.ranges.size(); i++)
	    // 	{
	    // 	    new_base.ranges.push_back(current_scan.ranges[i]);
	    // 	}
	    // 	new_base.transform = current_scan.transform * base_scan.transform;
	    // 	current_scan.transform << 1, 0, 0, 0,
	    // 	    0, 1, 0, 0,
	    // 	    0, 0, 1, 0,
	    // 	    0, 0, 0, 1;

	    // 	base_scan.copyFrom(&new_base);
	    //  }
    	}


>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
    }
};


int main (int argc, char** argv) {
<<<<<<< HEAD
    ros::init(argc, argv, "icp_odom");
=======
    ros::init(argc, argv, "line_finder");
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
    ros::NodeHandle nh;
	
    line_finder lf(nh);


    ros::Rate loop_rate(10);
    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}


<<<<<<< HEAD
void getTransform(vector<float3> base, 
		  vector<float3> target)
=======
void updateTransform(scan& base, 
		  scan& target)
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

<<<<<<< HEAD
    // Fill in the CloudIn data
    cloud_in->width    = base.size();
=======


    // Fill in the CloudIn data
    cloud_in->width    = base.ranges.size();
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
<<<<<<< HEAD
	cloud_in->points[i].x = base[i].x;
	cloud_in->points[i].y = base[i].y;
	cloud_in->points[i].z = base[i].z;
    }

    cloud_out->width    = target.size();
=======
	Vector4f ego_base;
	Vector4f allo_base;
	ego_base << base.ranges[i].x, base.ranges[i].y, base.ranges[i].z, 1;
	allo_base = target.transform * base.transform * ego_base;
	
	cloud_in->points[i].x = allo_base(0);
	cloud_in->points[i].y = allo_base(1);
	cloud_in->points[i].z = allo_base(2);
    }

    cloud_out->width    = target.ranges.size();
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
    cloud_out->height   = 1;
    cloud_out->is_dense = false;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);

    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
<<<<<<< HEAD
	cloud_out->points[i].x = target[i].x;
	cloud_out->points[i].y = target[i].y;
	cloud_out->points[i].z = target[i].z;
=======
	cloud_out->points[i].x = target.ranges[i].x;
	cloud_out->points[i].y = target.ranges[i].y;
	cloud_out->points[i].z = target.ranges[i].z;
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
    }

    std::cout << "base_size:" << cloud_in->points.size() << std::endl;
    std::cout << "target_size:" << cloud_out->points.size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
<<<<<<< HEAD
	//Ignore correspondenses that are greater than 5cm
	icp.setMaxCorrespondenceDistance(0.05);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
=======
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl; 
    //std::cout << icp.getFinalTransformation() << std::endl;
    target.transform *= icp.getFinalTransformation();
    std::cout << target.transform * base.transform << std::endl;
>>>>>>> d20e02e8b2fdcb88726843f7d748d8169f076b92
}
