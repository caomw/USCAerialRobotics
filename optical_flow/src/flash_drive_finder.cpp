#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <Eigen/Dense>

using namespace std;
using namespace cv;

class flash_drive_finder {

public:

	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> subImage;
	message_filters::Subscriber<geometry_msgs::PoseStamped> subAnnrduino;
	nn		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> Policy_sync_subs;
	message_filters::Synchronizer<Policy_sync_subs> sync_subs;
	uint subImageCount;
		
	Mat imgColor;
	Mat imgGray;
	Mat imgGrayPrevious;
	Mat imgCorner;
	float roll;
	float pitch;
	float yaw;
	float height;
	float rollPrevious;
	float pitchPrevious;
	float yawPrevious;
	float heightPrevious;


		

	FlashDriveFinderNode(ros::NodeHandle& _nh): nh(_nh), subImage(nh, "/camera/image_raw", 10), subArduino(nhn, "/arduino/attitude", 10), 
												sync_subs(Policy_sync_subs(10), subImage, subArduino)  
	{
		subImageCount = 0;
		imgColor = Mat(cv::Size(640, 480), CV_8UC3);
		imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
		imgGray = Mat(cv::Size(640, 480), CV_8UC1);
		
		sync_subs.registerCallback(boost::bind(&flash_drive_finder::sync_subs_callback, this, _1, _2));

		//Create a cv window to view image in for now
		cv::namedWindow("Image");
	}
	
	~FlashDriveFinderNode() {
		cv::destroyWindow("Image");
		imgColor.release();
		imgGray.release();
	}
	
	
	void sync_subs_callback(const sensor_msgs::Image::ConstPtr& msg, const geometry_msgs::PoseStamped::ConstPtr& msg_arduino)  {
		//if(subImageCount % 4 != 0) {
		//	subImageCount++;
		//	return;
		//}
		
	  	rollPrevious = roll;
		pitchPrevious = pitch;
		yawPrevious = yaw;
		heightPrevious = height;
		roll = msg_arduino->pose->orientation.x;
		pitch = msg_arduino->pose->orientation.y;
		yaw = = msg_arduino->pose->orientation.z;
		height = msg_arduino->pose->orientation.w;
		
		subImageCount++;
		cout << "Got image number: " << subImageCount << endl;
		
		
		//convert image from ROS image message to OpenCV Mat
		convert_image(msg, imgColor);
		
		//Do things with the image here
		
		//int numPyramidLevels = 0;
		//numPyramidLevels = buildOpticalFlowPyramid(prevImgGray, imgColor, Size winSize, int maxLevel, bool withDerivatives=true, int pyrBorder=BORDER_REFLECT_101, int derivBorder=BORDER_CONSTANT, bool tryReuseInputImage=true);
		

		vector<Point2f> *corners = new vector<Point2f>();
		cvtColor(imgColor, imgGray, CV_RGB2GRAY);
		
		if(subImageCount <= 1) {
			imgGrayPrevious = imgGray;
			return;
		}
		
		goodFeaturesToTrack(imgGrayPrevious, *corners, 100, .1, 10, Mat(), 3, true, .04);
		
		
		vector<Point2f> *nextPoints = new vector<Point2f>((*corners).size(), Point2f(0, 0));
		vector<uchar> status;
		vector<float> errors;
		

		calcOpticalFlowPyrLK(imgGrayPrevious, imgGray, *corners, *nextPoints, status, errors, Size(21,21), OPTFLOW_USE_INITIAL_FLOW, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.3), 0, .0001);
		
		for(int i = 0; i < corners->size(); i++) {
			printf("corners: x = %f y = %f  nextPts: x = %f y = %f\n", (*corners)[i].x, (*corners)[i].y, (*nextPoints)[i].x, (*nextPoints)[i].y);
		}
		
		cout << "Drawing circles..." << endl;
		for(unsigned int i = 0; i < corners->size(); i++) {
			cout << "point " << i << " x = " << (*corners)[i].x << "  y = " << (*corners)[i].y << endl;
			circle(imgColor, (*corners)[i], 8, cvScalar(255, 0, 0), 2, 8, 0);
		}
		
		cout << "Drawing lines..." << endl;
		cout << "corners size: " << corners->size() << "  nextpoints size: " << nextPoints->size() << endl;
		for(unsigned int i = 0; i < corners->size(); i++) {
		    //if the ith corner is not in bestMatches (returned from RANSAC), then don't display it, so that we can check which vectors are inliers within the square tolerance of our estimated motion. These should agree with our qualitative intuion.
			cout << i << endl;
			line(imgColor, (*corners)[i], (*nextPoints)[i], cvScalar(0, 0, 255), 1, 8, 0);
		}
		cout << "Done drawing" << endl;
		
		//Matches to Odometry Section

				
		//Display that image back onto the window
		cv::imshow("Image", imgColor);
		
		//Wait...
		cv::waitKey(1);
		imgGrayPrevious = imgGray.clone();
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

