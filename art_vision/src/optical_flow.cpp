#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/video/video.cpp>
#include <opencv2/features2d/features2d.cpp>
#include <opencv2/core/core.cpp>
#include <opencv2/highghui/highgui.cpp>

using boost::shared_ptr

class OpticalFlow
{
  ros::NodeHandle nh;

  ros::Subsciber sub_rawimage;
  uint counter_sub_rawiimage;

  cv::Mat image_old, img_new, img_result;
  shared_ptr<vector<cv::Point> > points_old, points_new;

 public:
  OpticalFlow (ros::NodeHandle& _nh): nh(_nh), counter_sub_rawimage(0)
  {
    sub_rawimage = nh.subscribe("/camera/image_rect", 20,
                                &OpticalFlow::callback_sub_rawimage, this);
    cv::namedWindows("result");
  }

  callback_sub_rawimage (const sensor_msgs::Image::ConstPtr& msg_rawimage)
  {
    // Only handle one msg in every five.
    if (counter_sub_rawimage % 5 != 0) return;

    // Convert the image into OpenCV format, and prepare
    image_new = cv::Mat(480, 640, CV_8UC1, (char*) msg->data[0]);
    
    // Do optical flow and PNP solving
    if (counter_sub_rawimage > 0) {
      
      // Find features for one image.
      cv::goodFeaturesToTrack(image_old, *points_old, 50, 0.1. 1);

      // Find corresponding feature for the other image.
      vector<char> status;
      cv::calcOpticalFlowPyrLK(image_old, image_new, *points_old, *points_new,
                               status, vector<int>);

      //cv::Vector3 tvec, rvec;
      //cv::solvePnPRansac(*points_old, *points_new, cv::eye(3), null, rvec, tvec);
    }

    image_src = image_new
    points_old = points_new;
    points_new.reset(new Vector<cv::Point>);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "optical_flow_node");
  ros::NodeHandle nh;
  OpticalFlow optical_flow(nh);
  ros::spin();
  return 0;
}

