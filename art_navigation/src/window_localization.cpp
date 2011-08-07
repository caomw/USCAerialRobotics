#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

using namespace std;

namespace art_navigation
{	
	class WindowNavigation : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		ros::Subscriber sub_kinect;
		ros::Publisher pub_points, pub_pose;
		
		struct PARAM_hough
		{
			int rho_res, theta_res, hough_thresh, min_length, max_gap;
			PARAM_hough(): rho_res(1), theta_res(0), hough_thresh(20), min_length(20), max_gap(8) {}
		} param_hough;

		struct PARAM_CANNY
		{
			int threshold1, threshold2;
			PARAM_CANNY(): threshold1(100), threshold2(200) {}
		} param_canny;

		struct Line
		{
			int loc_a[2], loc_b[2];
			float point_a[3], point_b[3];
			float length;
			float vector[3];
		};

	  private:
		virtual void onInit()
		{
			/// Init variables.
			
			/// Start messaging.
			nh = getNodeHandle();

			cv::namedWindow("haha");
			cv::createTrackbar("length", "haha", &standard_length, 400);
			
			pub_points = nh.advertise<sensor_msgs::PointCloud2>("/points", 10);
			pub_pose = nh.advertise<geometry_msgs::Pose2D>("/arduino/robot_pose", 10);
			sub_kinect = nh.subscribe("/kinect/raw", 10, &WindowPose::cb_sub_kinect, this);
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_navigation, window_pose, art_window::WindowPose, nodelet::Nodelet);
