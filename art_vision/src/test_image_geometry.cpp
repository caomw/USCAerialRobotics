#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>


class TestImageGeometry {

  ros::NodeHandle nh;

  sensor_msgs::CameraInfo info_cam;
  image_geometry::PinholeCameraModel model_cam;

public:
  TestImageGeometry (ros::NodeHandle& nh_): nh(nh_) {

    // TODO set up info cam
    


    model_cam.fromCameraInfo(info_cam);
  }

  void cb_image_raw (const sensor_msgs::Image::ConstPtr& msg_image_raw) {
    cv::Mat image_raw(480, 640, CV_8UC1, &(msg_image_raw->data[0]));

    sensor_msgs::PrintCloud cloud;

  }

}
