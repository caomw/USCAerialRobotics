#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

using namespace std;
using boost::shared_ptr;

class OpticalFlow {
public:
  ros::NodeHandle nh;
  ros::Subscriber sub_image_raw;
  ros::Publisher pub_cloud;
  size_t counter_sub_image_raw;
  
  cv::Mat image_old, image_new, image_result;
  shared_ptr<vector<cv::Point2f> > points_old, points_new;

  image_geometry::PinholeCameraModel model_cam;
  sensor_msgs::CameraInfo info_cam;

  tf::TransformListener tf_listener;

  OpticalFlow (ros::NodeHandle& _nh): nh(_nh), counter_sub_image_raw(0),
               points_old(new vector<cv::Point2f>()), points_new(new vector<cv::Point2f>()) {
    // Initialize variables.
    if (nh.getParam("camera_info", info_cam))
      model_cam.fromCameraInfo(info_cam);
    else {
      ROS_ERR("Cannot load camera info from parameter server."); return;
    }
    // Initialize messaging.
    sub_image_raw = nh.subscribe("/camera/image_mono", 10, &OpticalFlow::callback_sub_image_raw, this);
  }

  void callback_sub_image_raw (const sensor_msgs::Image::ConstPtr& msg_image_raw) {
    
    cv::Mat image_raw(480, 640, CV_8UC1, (void*) &(msg_image_raw->data[0]));
    cv::goodFeatures
    
    
    // Convert the image into OpenCV format, and prepare
    cv::resize(cv::Mat(480, 640, CV_8UC1, (char*) &(msg_image_raw->data[0])), image_new, cv::Size(320,240));
    
    // Do optical flow and PNP solving
    if (counter_sub_image_raw != 0) {
     
      // Find features for one image.
      cv::goodFeaturesToTrack(image_old, *points_old, 75, 0.25, 0.1);
     
      // Find corresponding feature for the other image.
      vector<uchar> status; vector<float> error;
      cv::calcOpticalFlowPyrLK(image_old, image_new, *points_old, *points_new, status, error);

      
      cv::imshow("result", image_result);
      cv::waitKey(2);
    }

    image_old = image_new;
    ++ counter_sub_image_raw;
  }
};

int main (int argc, char** argv) {
  ros::init(argc, argv, "optical_flow_node");
  ros::NodeHandle nh;
  OpticalFlow optical_flow(nh);
  ros::spin();
  return 0;
}

