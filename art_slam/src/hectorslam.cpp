/**
 * lrf_processor.cpp
 * Tal Levy, Aerial Robotics Team, USC
 * Receives and synchronizes gyro, laser range finder, and altitude 
 * data for estimating distance from walls. Marks potentially bad input
 * (ex. floor readings) based on altimeter
 */
#include <ros/ros.h>
#include <iomanip>
#include <string>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud.h>
