#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>


float angular_resolution = 0.3f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
ros::Publisher border_pub;

int threshold1   = 100;
int threshold2   = 200;
int rho_res      = 1;
int theta_res    = 5;
int hough_thresh = 28;
int min_length   = 60;
int max_gap      = 45;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
}

void depthimg_cb (const sensor_msgs::ImageConstPtr& input)
{
  std::cout << "Got A Depth Image" << std::endl;
  cv::Mat img;
  img.create(input->height, input->width, CV_8UC1);
  short* data_ptr = (short*)(&input->data[0]);
  for(int i = 0; i <= input->height * input->width; i++)
  {
    if(data_ptr[i] <= 2048/* threshold */)
      img.data[i] = data_ptr[i] / 8;
    else
      img.data[i] = 0;
  }

  cv::Mat morphImg;

  cv::morphologyEx(img, morphImg, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 5);

  // Find the edges in the depth image
  cv::Mat edges;
  cv::Canny(morphImg, edges, threshold1, threshold2);


  cv::Mat dispImg;
  cv::cvtColor(edges, dispImg, CV_GRAY2BGR);

  //use this edge image to find some straight lines using a probabalistic Hough transform
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, std::max(0.1,double(rho_res)), std::max(0.1,theta_res * M_PI/180.0), std::max(1.0,double(hough_thresh)),
    std::max(1, min_length), std::max(1, max_gap));
  for( size_t i = 0; i < lines.size(); i++ )
  {
    if(lines[i][0] < 30 || lines[i][0] > img.cols-30) continue;
    if(lines[i][1] < 30 || lines[i][1] > img.rows-30) continue;
    if(lines[i][2] < 30 || lines[i][2] > img.cols-30) continue;
    if(lines[i][3] < 30 || lines[i][3] > img.rows-30) continue;
    cv::line(dispImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(rand()%255,rand()%255,rand()%255), 3, 8);
  }

  // Let's look for corners too!
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(edges, corners, 50, .50, 10);
  for(int i=0; i<corners.size(); ++i)
    cv::circle(dispImg, corners[i], 4, cv::Scalar(0, 0, 255));

  cv::imshow("img", morphImg);
  cv::imshow("edges", dispImg);
  cv::waitKey(100);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "art_window");
  ros::NodeHandle nh;

  cv::namedWindow("img");
  cv::namedWindow("edges");
  cv::createTrackbar("threshold1",      "edges", &threshold1,   255);
  cv::createTrackbar("threshold2",      "edges", &threshold2,   255);
  cv::createTrackbar("rho_res (pix)",   "edges", &rho_res,      255);
  cv::createTrackbar("theta_res (deg)", "edges", &theta_res,    360);
  cv::createTrackbar("min_length",      "edges", &min_length,   255);
  cv::createTrackbar("max_gap",         "edges", &max_gap,      255);
  cv::createTrackbar("hough thresh",    "edges", &hough_thresh, 255);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber points_sub = nh.subscribe ("/points", 1, cloud_cb);

  ros::Subscriber img_sub = nh.subscribe ("/kinect/depth", 1, depthimg_cb);

  border_pub = nh.advertise<sensor_msgs::PointCloud2>("border_points", 1);


  // Spin
  ros::spin ();
}

