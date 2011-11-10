
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <cv.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;

typedef cv::Mat Mat;

using cv::Mat;

class Downward_Lasers
{	
  public:
	ros::NodeHandle nh;
	ros::Subscriber sub_image;
	uint sub_image_count;
	Mat img_color;
	Mat img_gray;
	vector<cv::Vec3f> circles;
	
	
  public:

	Downward_Lasers(ros::NodeHandle& _nh)//: nh(_nh), sub_image_count(0), img_color(cv::Size(640, 480), cv::CV_8UC3), img_gray(cv::Size(640, 480), cv::CV_8UC3)
	{
		nh = _nh;
		img_color = Mat(cv::Size(640, 480), CV_8UC3);
		img_gray = Mat(cv::Size(640, 480), CV_8UC3);


		sub_image = nh.subscribe("/usb_cam/image_raw", 10, &Downward_Lasers::sub_image_callback, this);		
		cv::namedWindow("test_result");
	}
	
	~Downward_Lasers()
	{
		cv::destroyWindow("test_result");
		img_color.release();
		img_gray.release();
	}
	
	void sub_image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if(sub_image_count % 10 == 0)
		{
			convert_image(msg, img_color);
			
			//TODO- filter the image

			//convert to greyscale and smooth to avoid multiple flase circles in close proximity
			cv::cvtColor(img_color, img_gray, CV_BGR2GRAY);
			cv::GaussianBlur( img_gray, img_gray, cv::Size(9, 9), 2, 2);

			//get circles
			cv::HoughCircles(img_gray, circles, CV_HOUGH_GRADIENT, 2, img_gray.rows/4, 100, 100, 0, 4 ); 

			//findHomography(Mat(points_matches_src), Mat(points_matches_tgt), mask_outliers, CV_RANSAC, 3);
				
			//Draw circles on screen
			for(int i = 0; i < circles.size(); i++)
			{
				cv::circle(img_color, cv::Point2f(circles[i][0], circles[i][1]), circles[i][3], cv::Scalar(255,0,0), 1, 8, 0);
			}				

			cv::imshow("test_result", img_color);
			//cv::waitKey(0);
		}
		cout << sub_image_count << endl;
		++ sub_image_count;
	}

  private:

	void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		_img = cv_image_ptr->image;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_downward_facing_lasers");
	ros::NodeHandle nh;
	
	Downward_Lasers downward_lasers(nh);
	
	ros::spin();
	return 0;
}
