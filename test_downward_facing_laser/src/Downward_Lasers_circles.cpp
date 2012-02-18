
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
	
	Mat img_thresh_low;
	Mat img_thresh_high;
	
	
  public:

	Downward_Lasers(ros::NodeHandle& _nh)//: nh(_nh), sub_image_count(0), img_color(cv::Size(640, 480), cv::CV_8UC3), img_gray(cv::Size(640, 480), cv::CV_8UC3)
	{
		nh = _nh;
		img_color = Mat(cv::Size(640, 480), CV_8UC3);
		img_gray = Mat(cv::Size(640, 480), CV_8UC3);

		sub_image_count = 0;
		sub_image = nh.subscribe("/usb_cam/image_raw", 10, &Downward_Lasers::sub_image_callback, this);		
		cv::namedWindow("test_result");
		cv::namedWindow("img_color");
		
		

	}
	
	~Downward_Lasers()
	{
		cv::destroyWindow("test_result");
		cv::destroyWindow("img_color");
		img_color.release();
		img_gray.release();
	}
	
	void sub_image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if(sub_image_count % 4 == 0)
		{
			convert_image(msg, img_color);
			
			//TODO- filter the image

			//convert to greyscale and smooth to avoid multiple flase circles in close proximity

			//cv::inRange(img_color, cv::Scalar(255,255,255), cv::Scalar(10,230,10), img_gray);
			//cv::inRange(img_color, cv::Scalar(280), cv::Scalar(200), img_color);


			cv::GaussianBlur( img_color, img_color, cv::Size(9, 9), 2, 2);
//-

			Mat hsv_frame = Mat(cv::Size(640, 480), CV_8UC3);
			Mat img_thresholded = Mat(cv::Size(640, 480), CV_8UC1);
			Mat thresholded2 = Mat(cv::Size(640, 480), CV_8UC1);
			cv::Scalar hsv_min = cv::Scalar(65, 100, 170, 0);	//0
			cv::Scalar hsv_max = cv::Scalar(95, 256, 256, 0);	//10
			//cv::Scalar hsv_min2 = cv::Scalar(170, 50, 170, 0);
			//cv::Scalar hsv_max2 = cv::Scalar(256, 180, 256, 0);

			// convert to HSV for color matching
			//as hue wraps around, we need to match it in 2 parts and OR together
			cv::cvtColor(img_color, hsv_frame, CV_BGR2HSV);
			cv::inRange(hsv_frame, hsv_min, hsv_max, img_thresholded);
			//cv::inRange(hsv_frame, hsv_min2, hsv_max2, thresholded2);
			//v::bitwise_or(img_thresholded, thresholded2, img_thresholded);
			
			
			
			cv::circle(img_color, cv::Point2f(100, 100), 25, cv::Scalar(255,0,0), 1, 8, 0);

			//I(x,y)hue ~ ((uchar*)(img->imageData + img->widthStep*y))[x*3]
			//I(x,y)sat ~ ((uchar*)(img->imageData + img->widthStep*y))[x*3+1]
			//I(x,y)val ~ ((uchar*)(img->imageData + img->widthStep*y))[x*3+2]

			int x = 100, y = 100;

			cout << "HSV: " << endl;
			cout << (uint) (img_color.data + img_color.step*y)[x*3] << endl;
			cout << (uint) (img_color.data + img_color.step*y)[x*3+1] << endl;
			cout << (uint) (img_color.data + img_color.step*y)[x*3+2] << endl;

//-
			//get circles
			cv::HoughCircles(img_thresholded, circles, CV_HOUGH_GRADIENT, 2, img_thresholded.rows/4, 100, 100, 0, 20 );

			cout << "numcircles: " << circles.size() << endl;
				
			//Draw circles on screen
			for(int i = 0; i < circles.size(); i++)
			{
				cv::circle(img_thresholded, cv::Point2f(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(255,0,0), 1, 8, 0);
				cout << "r: " << circles[i][2] << endl;
			}				

			cv::imshow("test_result", img_thresholded);
			cv::imshow("img_color", img_color);
			cv::waitKey(1);
		}
		//cout << sub_image_count << endl;
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
