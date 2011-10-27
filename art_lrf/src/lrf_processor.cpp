
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
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;

const string LRFTopic = "scan";
const string GyroTopic = "/gyro";

class LRF_Processor
{t
  public:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_lrf;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_gyro;
    //message_filters::Subscriber<std::_msgs::Float> sub_alt;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, 
                      geometry_msgs::Vector3Stamped> Policy_sync_subs;
    message_filters::Synchronizer<Policy_sync_subs> sync_subs;
    
    LRF_Processor(ros::NodeHandle& _nh): nh(_nh),
        sub_lrf(nh, LRFTopic, 10),
        sub_gyro(nh, GyroTopic, 10),
        sync_subs(Policy_sync_subs(10),sub_lrf, sub_gyro)
    {
        cout << "test";
        sync_subs.registerCallback(boost::bind(&LRF_Processor::sync_subs_callback, this, _1, _2));
        sub_lrf.registerCallback(&LRF_Processor::sub_lrf_callback, this);
    }
    
    void sub_lrf_callback(const sensor_msgs::LaserScan::ConstPtr& _msg_lrf)
    {
	  float min = _msg_lrf->angle_min;
      for (unsigned int i = 0; i < sizeof(_msg_lrf->ranges)/sizeof(_msg_lrf->ranges.at(0)); i++){
        float angle = min + _msg_lrf->angle_increment;
        float range = _msg_lrf->ranges[i];
        float mid_angle = (min + _msg_lrf->angle_max) / 2;
        cout << angle << '\t' << range << endl;
        float x_coord = range * sin(mid_angle - angle);
        float y_coord = range * cos(mid_angle - range);
        cout << "\t\t\t\t\t(" << x_coord << ", " << y_coord << endl;
      }
	}
    
    /*Callback receives time-synced Laser Scan, Gyro, and altitude data.
     * Processes data and returns output as a set of (x, y) data points,
     * each data point is determined to be a wall or a floor based on the
     * altitude data
     */
    void sync_subs_callback(const sensor_msgs::LaserScan::ConstPtr& _msg_lrf,
                            const geometry_msgs::Vector3Stamped::ConstPtr& _msg_gyro
                            )
    {
      float min = _msg_lrf->angle_min;
      for (unsigned int i = 0; i < sizeof(_msg_lrf->ranges)/sizeof(_msg_lrf->ranges.at(0)); i++){
        float angle = min + _msg_lrf->angle_increment;
        float range = _msg_lrf->ranges[i];
        float mid_angle = (min + _msg_lrf->angle_max) / 2;
        cout << angle << '\t' << range << endl;
        float x_pitch = _msg_gyro->vector.x;
        float y_roll = _msg_gyro->vector.y;
        double x_coord = range * sin(mid_angle - angle);
        double y_coord = range * cos(mid_angle - range);
        cout << "\t\t\t\t\t(" << x_coord << ", " << y_coord << endl;
        double z_displacement = x_coord * sin(x_pitch);
        x_coord *= cos(x_pitch);
        y_coord *= cos(y_roll);
        
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "processor");
  ros::NodeHandle nh;
  
  cout << "test";
  LRF_Processor lrf_processor(nh);
  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
	  ros::spinOnce();
	  loop_rate.sleep();
  }
  return 0;
}
