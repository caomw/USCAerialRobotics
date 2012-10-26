#include <ros/ros.h>
#include <tf/transform_listener.h>

#define ToDeg(x) x / 2 / 3.14159265358979 * 360

using namespace std;

int main (int argc, char** argv) {
  ros::init(argc, argv, "msg_arduino_node");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;

  ros::Rate rate(10.0);

  while(ros::ok()) {
    ros::Time time_now = ros::Time::now();
    tf::StampedTransform transform;
    //tf_listener.waitForTransform("/world", "/imu", time_now, ros::Duration(0.1));
    try {
      tf_listener.lookupTransform("/world", "/imu", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      continue;
    }
    double roll, pitch, yaw;
    unsigned int dummy = 1;
    transform.getBasis().getEulerYPR(yaw, pitch, roll, dummy);
    cout << "get new: " << ToDeg(roll) << "\t" << ToDeg(pitch) << "\t" << ToDeg(yaw) << endl;

    rate.sleep();
  }
  return 0;
}
