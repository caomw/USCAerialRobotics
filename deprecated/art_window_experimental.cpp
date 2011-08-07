#include <ros/ros.h>
#include <art_common/KinectMsg.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

/// Constants for HoughLinesP
int rho_res = 1, theta_res = 0, hough_thresh = 20, min_length = 40, max_gap = 8;

int threshold1 = 50, threshold2 = 200;

ros::Subscriber sub_kinect;
ros::Publisher pub_points;
ros::Publisher pub_pose;

static const float centerX = 159.5, centerY = 119.5, constant = 1.0 / 262.5;

void get_3d_point(const art_common::KinectMsg::ConstPtr msg, float* point, int x, int y)
{
	float Z = static_cast<float>((reinterpret_cast<const short*>(&msg->depth[0]))[(y + 15) * 320 + (x + 15)]) * 0.001;
	point[0] = (x - centerX) * Z * constant;
	point[1] = (y - centerY) * Z * constant;
	point[2] = Z;
}

bool get_closest_point(cv::Mat img, int x, int y, int& x_closest, int& y_closest, int blocksize)
{
	bool found = false;
	x_closest = 0;
	y_closest = 0;
	int square = 0;
	for(int i = blocksize; i >= - blocksize; i--)
	{
		for(int j = - blocksize; j <= blocksize; j++)
		{
			if(img.at<uchar>(y+j,x+i) == 255)
				if((!found) || (j*j+i*i < square))
					{ x_closest = i; y_closest = j; square = j*j+i*i; found = true;}
		}
	}
	return found;
}

void cb_sub_kinect(const art_common::KinectMsg::ConstPtr msg)
{
	cv::Mat img(210, 290, CV_8UC1, cv::Scalar(0));

	/// Crop the original image.
	{
		short* msg_ptr = (short*) &(msg->depth[0]) + 320 * 15 + 15;
		int img_ptr = 0;
		for(int i = 0; i < 210; i++)
		{
			for(int j = 0; j < 290; j++, msg_ptr++, img_ptr++)
				if(*msg_ptr < 5120) img.data[img_ptr] = (uchar) (*msg_ptr / 20);
			msg_ptr += 30;
		}
	}

	/// Border detection.
	cv::Mat edges, edges_raw;
	cv::Canny(img, edges_raw, threshold1, threshold2);

	cv::blur(edges_raw, edges, cv::Size(4,4));
	cv::threshold(edges, edges_raw, 0, 255, cv::THRESH_BINARY);

	pcl::PointCloud<pcl::PointXYZ> points;
	for(int y = 0; y < 210; y++)
	{
		for(int x = 0; x < 290; x++)
		{
			if(img.at<uchar>(y,x) != 0)
			{
				float point[3];
				get_3d_point(msg, point, x, y);
				points.points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
			}
		}
	}

	sensor_msgs::PointCloud2 points_msg;
	pcl::toROSMsg(points, points_msg);
	points_msg.header.frame_id = "/world";
	points_msg.header.stamp = ros::Time::now();
	pub_points.publish(points_msg);

	cv::Mat dispImg(210, 290, CV_8UC3, cv::Scalar(0,0,0));
	//cv::cvtColor(edges_raw, dispImg, CV_GRAY2BGR);
	
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(edges_raw, lines, std::max(0.1,double(rho_res)), std::max(0.1,theta_res * M_PI/180.0), std::max(1.0,double(hough_thresh)), std::max(1, min_length), std::max(1, max_gap));
	for(size_t i = 0; i < lines.size(); i++)
	{
		cv::line(dispImg, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(200,200,200), 3, 8);
	}


	
	cv::imshow("edges", dispImg);
	cv::waitKey(50);
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "art_window");
	ros::NodeHandle nh;
	cout << endl;

	cv::namedWindow("edges");
	cv::createTrackbar("threshold1", "edges", &threshold1, 255);
	cv::createTrackbar("threshold2", "edges", &threshold2, 255);

	pub_points = nh.advertise<sensor_msgs::PointCloud2>("/points", 10);
	pub_pose = nh.advertise<geometry_msgs::Pose2D>("/arduino/robot_pose", 10);
	sub_kinect = nh.subscribe("/kinect/raw", 10, cb_sub_kinect);

	ros::spin();
	return 0;
}
