#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf/transform_broadcaster.h>

using namespace std;

class CheckerBoard
{
  ros::NodeHandle nh;
  ros::Subscriber sub_image_raw;
  uint counter_sub_image_raw;
  tf::TransformBroadcaster br_trafo;
  tf::Transform trafo;

  public:
  CheckerBoard (ros::NodeHandle& _nh): nh(_nh), counter_sub_image_raw(0) {
    sub_image_raw = nh.subscribe("/camera/image_rect", 100,
        &CheckerBoard::callback_sub_image_raw, this);
  }

  void callback_sub_image_raw (const sensor_msgs::Image::ConstPtr& msg_image_raw) {

    // Resize picture.
    cv::Mat image_now;
    cv::resize(cv::Mat(480, 640, CV_8UC1, (char*) &(msg_image_raw->data[0])),
        image_now, cv::Size(320, 240));


    // Detech checkerboard.
    vector<cv::Point2f> points_img;
    if (counter_sub_image_raw % 6 == 0) {
      if (cv::findChessboardCorners(image_now, cv::Size(8,6), points_img)) {
        drawChessboardCorners(image_now, cv::Size(8,6), points_img, true);
        cv::imshow("lol", image_now);
        cv::waitKey(2);


        // Build 3D model.
        vector<cv::Point3f> points_obj;
        for (size_t i = 0; i < 6; i++) for (size_t j = 0; j < 8; j++)
          points_obj.push_back(cv::Point3f(i, 7 - j,0));

        // Build camera matrix.
        float data_mat_intrinsic[9] = {650.205744629443, 0, 300.450354600291, 0,
                                       648.217983879262, 247.490523265453, 0, 0, 1};
        cv::Mat mat_intrinsic(3, 3, CV_32FC1, data_mat_intrinsic);
        cv::Mat mat_distortion(5,1,CV_32FC1, cv::Scalar(0));
        
        // Do PNP
        cv::Mat rvec, tvec;
        cv::solvePnPRansac(points_obj, points_img, mat_intrinsic, mat_distortion, rvec, tvec);

        trafo.setOrigin(tf::Vector3(tvec.at<double>(0)/10, tvec.at<double>(1)/10,
                        tvec.at<double>(2)/10));
        trafo.setRotation(tf::Quaternion(rvec.at<double>(2), rvec.at<double>(1),
                          rvec.at<double>(0)));
        br_trafo.sendTransform(tf::StampedTransform(trafo, ros::Time::now(),
                               "/world", "/camera"));
      }
    }

    counter_sub_image_raw ++;
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "checker_board_node");
  ros::NodeHandle nh;
  CheckerBoard checker_board(nh);
  ros::spin();
  return 0;
}

