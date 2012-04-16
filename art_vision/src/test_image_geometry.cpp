#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

using namespace std;

class TestImageGeometry {

  ros::NodeHandle nh;
  ros::Publisher pub_cloud;
  image_transport::ImageTransport it;
  image_transport::CameraSubscriber sub_image_raw;
  image_geometry::PinholeCameraModel model_cam;
  tf::TransformListener tf_listener;

public:
  TestImageGeometry (ros::NodeHandle nh_): nh(nh_), it(nh) {
    pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/cloud", 10);
    sub_image_raw = it.subscribeCamera("image_raw", 10,
                                       &TestImageGeometry::cb_sub_image_raw,
                                       this);
  }

  void cb_sub_image_raw (const sensor_msgs::ImageConstPtr& msg_image_raw,
                         const sensor_msgs::CameraInfoConstPtr& msg_info_cam) {
    cv::Mat image_raw(480, 640, CV_8UC1, (void*) &(msg_image_raw->data[0]));
    model_cam.fromCameraInfo(msg_info_cam);

    tf::StampedTransform trafo;
    ros::Time time_on = ros::Time::now();
    ros::Duration time_out(0.5);
    tf_listener.waitForTransform("/world", "/camera", time_on, time_out);
    tf_listener.lookupTransform("/world", "/camera", time_on, trafo);
    
    vector<cv::Point2f> pixels;
    initialize(pixels);

    sensor_msgs::PointCloud cloud;
    for (size_t i = 0; i < pixels.size(); i++) {
      cv::Point3f point = model_cam.projectPixelTo3dRay(model_cam.rectifyPoint(pixels[i]));
      
      btVector3 vec_point(point.x, point.y, point.z);
      vec_point = trafo * vec_point;
      geometry_msgs::Point32 msg_point;
      msg_point.x = vec_point.getX();
      msg_point.y = vec_point.getY();
      msg_point.z = vec_point.getZ();

      msg_point.x *= 2/msg_point.z;
      msg_point.y *= 2/msg_point.z;
      msg_point.z = 2;

      cloud.points.push_back(msg_point);
    }
    cloud.header.frame_id="/world";
    pub_cloud.publish(cloud);

    /*
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(image_raw, corners, 50, 0.5, 0.1);
    for(size_t i = 0; i < corners.size(); i++)
      cv::circle(image_raw, corners[i], 3, cv::Scalar(255), 3);
    cv::imshow("yes", image_raw);
    cv::waitKey(10);
    */
  }


private:
  void initialize (vector<cv::Point2f>& pixels) {
    for (size_t x = 0; x < 640; x += 10) {
      for (size_t y = 0; y < 480; y += 10) {
        pixels.push_back(cv::Point2f(x,y));
      }
    }
  }
};

int main (int argc, char** argv) {
  ros::init(argc, argv, "node_test_image_geometry");
  ros::NodeHandle nh;
  TestImageGeometry test(nh);
  ros::spin();
  return 0;
}
