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
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

const string LRFTopic = "/laserrf";
const string GyroTopic = "/gyro";

class LRF_Processor
{
  public:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_lrf;
    message_filters::Subscriber<geometry_msgs::Vector3> sub_gyro;
    //message_filters::Subscriber<std::_msgs::Float> sub_alt;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, 
                      geometry_msgs::Vector3> Policy_sync_subs;
    message_filters::Synchronizer<Policy_sync_subs> sync_subs;
    
    LRF_Processor(ros::NodeHandle* _nh): nh(_nh),
        sub_lrf(nh, LRFTopic, 10),
        sub_gyro(nh, GyroTopic, 10),
        sync_subs(Policy_sync_subs(10),sub_lrf, sub_gyro)
    {
        sync_subs.registerCallback(boost:bind(&LRF_Processor::sync_subs_callback);
    }
    
    /*Callback receives time-synced Laser Scan, Gyro, and altitude data.
     * Processes data and returns output as a set of (x, y) data points,
     * each data point is determined to be a wall or a floor based on the
     * altitude data
     */
    void sync_subs_callback(const sensor_msgs::LaserScan::ConstPtr& _msg_lrf,
                            const geometry_msgs::Vector3::ConstPtr& _msg_gyro
                            )
    {
      float32 min = _msg_lrf.angle_min;
      for (int i = 0; i < sizeof(_msg_lrf.ranges)/sizeof(_msg_lrf.ranges*); i++){
        cout << min + _msg_lrf.angle_increment << '\t' << _msg_lrf.ranges[i] << endl;
      }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lrf_processor");
  ros::NodeHandle nh;
  
  LRF_Processor lrf_processor(nh);
  
  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
	  ros.spinOnce();
	  d.sleep();
  }
  return 0;
}
