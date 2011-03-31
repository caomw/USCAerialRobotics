//      exatmple.cpp
//      An exapmle program testing SURF matching algorithm.
//
//      Copyright (C) 2011 Sam (Yujia Zhai) <yujia.zhai@usc.edu>
//      Aerial Robotics Team, USC Robotics Society - http://www.uscrs.org - http://uscrs.googlecode.com
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

// You can copy this header or click Geany menu "Edit-> Insert Comments-> Insert File Header".
// A copy of GNU GPL 2 lisense should always be in the root folder of your package.

// Use < > for including library files.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Use " " for including header files in the package.
#include "debug.hpp"

// Try to use as few using as possible to avoid conflicts and confusion.
using namespace std;
using art::Timer;

// Instead, use typedef for certain types. Geany editor can also highlight your typedef automatically.
// You can also put typedef in header files. Geany will also hightlight for you when the header file is opened.
typedef cv::Mat Mat;

class Example
{	
	ros::NodeHandle nh;
	ros::Subscriber sub_image;
	uint sub_image_count;
	Mat img_src, img_tgt, img_result;
	cv::SurfFeatureDetector surf_feature_detector;
	cv::SurfDescriptorExtractor surf_descriptor_extractor;
	
	// OpenCV provides a class for smart pointer too, but the one from boost library is more widely used throughout the ROS.
	// Example: create: Ptr<variable_type> variable_name; delete object: variable_name.reset();
	// Check out http://www.boost.org/doc/libs/1_46_1/libs/smart_ptr/shared_ptr.htm for more info.
	boost::shared_ptr< vector<cv::KeyPoint> > keypoints_src_ptr, keypoints_tgt_ptr;
	
	Mat descriptors_src, descriptors_tgt;
	
  public:

	Example(ros::NodeHandle& _nh): nh(_nh), sub_image_count(0), img_src(cv::Size(640, 480), CV_8UC3), img_tgt(cv::Size(640, 480), CV_8UC3), img_result(cv::Size(1280, 480), CV_8UC3)
	{
		sub_image = nh.subscribe("/usb_cam/image_raw", 10, &Example::sub_image_callback, this);		
		cv::namedWindow("test_result");
	}
	
	~Example()
	{
		// Release the pointers.
		cv::destroyWindow("test_result");
		img_src.release();
		img_tgt.release();
		img_result.release();
		keypoints_src_ptr.reset();
		keypoints_tgt_ptr.reset();
	}
	
	void sub_image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// Only handle one msg in every five.
		if(sub_image_count % 5 == 0)
		{
			// Let img_tgt point to the new image, while img_src keeps pointing to the previous image.
			convert_image(msg, img_tgt);

			Timer t;
			t.reset();

			// Extract features and get keypoints as well as descriptors.
			keypoints_tgt_ptr.reset(new vector<cv::KeyPoint>);
			surf_feature_detector.detect(img_tgt, *keypoints_tgt_ptr);
			surf_descriptor_extractor.compute(img_tgt, *keypoints_tgt_ptr, descriptors_tgt);
			
			t.echospan("Time used by SURF feature processing";)
			
			// Draw a circle for every keypoint on image.
			BOOST_FOREACH(cv::KeyPoint kp, *keypoints_tgt_ptr)
			{
				cv::circle(img_tgt, kp.pt, 5, cv::Scalar(255,0,0));
			}
			
			if(sub_image_count == 0) // First run.
			{
				// TODO
			}
			else
			{	
				// Composing the result image.
				Mat roi_1(img_result, cv::Rect(0, 0, 640, 480));
				img_src.copyTo(roi_1);
				Mat roi_2(img_result, cv::Rect(640, 0, 640, 480));
				img_tgt.copyTo(roi_2);
				roi_1.release();
				roi_2.release();
				
				// Show the result image.
				cv::imshow("test_result", img_result);
				cv::waitKey(5);
				
			}
			
			// Let src pointing to the tgt (original data stored in src will be discarded automatically).
			img_src = img_tgt;
			keypoints_src_ptr = keypoints_tgt_ptr;
			descriptors_src = descriptors_tgt;
		}
		
		// Increase the counter.
		++ sub_image_count;
	}

  private:

	// Convert the Ros Image message to OpenCV Mat.
	void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		_img = cv_image_ptr->image;
	}
};

int main(int argc, char** argv)
{
	//Initialize ROS node.
	ros::init(argc, argv, "programming_standard_example");
	ros::NodeHandle nh;
	
	//Run main functionality. You may define and run several classes if necessary.
	Example example(nh);
	
	ros::Rate loop_rate(10); //Spin the program. Rate is in Hertz. You can also use ros::Duration (float second).
	while(ros::ok())
	{	
		//
		// TODO: the loop body.
		// (if you have nothing to do during the loop, you can delete this "while()" as well as "ros::Rate" and call "ros::Spin()" instead.)
		//
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
