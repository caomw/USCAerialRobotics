#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class Rectify3D {
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> sub_image_raw;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_rotation;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    geometry_msgs::Vector3Stamped> Policy_sync_subs;
  message_filters::Synchronizer<Policy_sync_subs> sync_subs;

 public:
  Rectify3D (ros::NodeHandle& _nh): nh(_nh),
                                    sub_image_raw(nh, "/camera/image_mono", 10),
                                    sub_rotation(nh, "/angle", 10),
                                    sync_subs(Policy_sync_subs(10), sub_image_raw, sub_rotation)
  {
    sync_subs.registerCallback(boost::bind(&Rectify3D::callback_sync_subs, this, _1, _2));
  }

  void callback_sync_subs (const sensor_msgs::Image::ConstPtr& msg_image_raw,
                           const geometry_msgs::Vector3Stamped::ConstPtr& msg_rotation) {
    
    cv::Mat image_now;
    cv::resize(cv::Mat(480, 640, CV_8UC1, (char*) &(msg_image_raw->data[0])), image_now, cv::Size(240, 320));

    // Get features from image.
    vector<cv::Point> points_now;
    cv::goodFeaturesToTrack(image_now, points_now, 50, 0.1, 1);
    
    sensor_msgs::PointCloud pointcloud;
    
    // Get x, y and z from r, p and assume d, f;
    float f = 1.0, d = 1.0;
    for (int i = 0; i < points_now.size(); i++) {
      geometry_msgs::Point32 point;
      point.z = d / (1 - (points_now[i].x * tan(msg_rotation->vector.x) / f) +
                          points_now[i].y * tan(msg_rotation->vector.y) / f);
      point.x = points_now[i].x * point.z / f;
      point.y = points_now[i].y * point.z / f;
      pointcloud.points.push_back(point);
    }
  }


};


int main (int argc, char** argv) {
  ros::init(argc, argv, "rectify_3d_node");
  ros::NodeHandle nh;
  Rectify3D rectify_3d(nh);
  
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
