#include <ros/ros.h>
#include <cmath>
#include <string>
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
using namespace cv;

class FlashDriveFinderNode {

	public:

	private:
		ros::NodeHandle nh;
		ros::Subscriber subImage;
		uint subImageCount;
		
		Mat imgColor;

	public:
	
	FlashDriveFinderNode(ros::NodeHandle& _nh) {
		this->nh = _nh;
		
		subImageCount = 0;
		imgColor = Mat(cv::Size(640, 480), CV_8UC3);
		
		//subscribe to the image
		subImage = nh.subscribe("/camera/image_raw", 10, &FlashDriveFinderNode::subImageCallback, this);

		//Create a cv window to view image in for now
		cv::namedWindow("Image");

	}
	
	~FlashDriveFinderNode() {
		cv::destroyWindow("Image");
		imgColor.release();
	}
	
	private:
	
	void subImageCallback(const sensor_msgs::ImageConstPtr& msg) {
		subImageCount++;
		cout << "Got image number: " << subImageCount << endl;
		
		//convert image from ROS image message to OpenCV Mat
		convert_image(msg, imgColor);
		
		//Do things with the image here
		
		//Display that image back onto the window
		cv::imshow("Image", imgColor);
		
		//Wait...
		cv::waitKey(1);
	}

	void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		_img = cv_image_ptr->image;
	}
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "flash_drive_finder");
	ros::NodeHandle nh;
	
	cout << "Initializing FlashDriveFinderNode..." << endl;
	FlashDriveFinderNode flashDriveFinderNode(nh);
	cout << "Initialization Complete" << endl;
	
	ros::spin();
	return 0;
}

