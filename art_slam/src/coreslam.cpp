/**
 * lrf_processor.cpp
 * Tal Levy, Aerial Robotics Team, USC
 * Testing SLAM based on lrf reading values
 */
#include <ros/ros.h>
#include <iomanip>
#include <string>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

const String cloudTopic = "/lrf/PointCloud";

int main(int argc, char **argv){
	ros::init(argc, argv, "processor");
	ros::NodeHandle nh;
	
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}
