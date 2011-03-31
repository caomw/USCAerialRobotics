//      test_feature_match.cpp
//      Feature matching algorithm based on SURF and RANSAC (pyramid may be used in the future).
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

// Super-accurate feature matching based on SURF, homography and Random Sample Consensus.
// Could be improved by:
//   matching knn results instead of only the best, and
//   inspecting the relationship between homography matrix and transformation matrix from point cloud SVD.

#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;

typedef cv::Mat Mat;

class Test_Surf_Match
{	
	ros::NodeHandle nh;
	ros::Subscriber sub_image;
	uint sub_image_count;
	Mat img_src, img_tgt, img_result;
	cv::SurfFeatureDetector surf_feature_detector;
	cv::SurfDescriptorExtractor surf_descriptor_extractor;
	cv::FlannBasedMatcher descriptor_matcher;
	
	boost::shared_ptr< vector<cv::KeyPoint> > keypoints_src_ptr, keypoints_tgt_ptr;	
	Mat descriptors_src, descriptors_tgt;
	vector<cv::DMatch> match_result;
	
  public:

	Test_Surf_Match(ros::NodeHandle& _nh): nh(_nh), sub_image_count(0), img_src(cv::Size(640, 480), CV_8UC3), img_tgt(cv::Size(640, 480), CV_8UC3), img_result(cv::Size(640, 960), CV_8UC3)
	{
		sub_image = nh.subscribe("/usb_cam/image_raw", 10, &Test_Surf_Match::sub_image_callback, this);		
		cv::namedWindow("test_result");
	}
	
	~Test_Surf_Match()
	{
		cv::destroyWindow("test_result");
		img_src.release();
		img_tgt.release();
		img_result.release();
		keypoints_src_ptr.reset();
		keypoints_tgt_ptr.reset();
	}
	
	void sub_image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if(sub_image_count % 10 == 0)
		{
			convert_image(msg, img_tgt);

			keypoints_tgt_ptr.reset(new vector<cv::KeyPoint>);
			surf_feature_detector.detect(img_tgt, *keypoints_tgt_ptr);
			surf_descriptor_extractor.compute(img_tgt, *keypoints_tgt_ptr, descriptors_tgt);
			
			// draw circle on features
			BOOST_FOREACH(cv::KeyPoint kp, *keypoints_tgt_ptr)
			{
				cv::circle(img_tgt, kp.pt, 5, cv::Scalar(255,0,0));
			}
			
			if(sub_image_count == 10)
			{
				
			}
			else if(sub_image_count == 20)
			{	
				Mat roi_1(img_result, cv::Rect(0, 0, 640, 480));
				img_src.copyTo(roi_1);
				Mat roi_2(img_result, cv::Rect(0, 480, 640, 480));
				img_tgt.copyTo(roi_2);
				roi_1.release();
				roi_2.release();
				
				descriptor_matcher.match(descriptors_src, descriptors_tgt, match_result);
				
				// TODO: Cross Matching check
				
				vector<int> indices_matches_src(match_result.size()), indices_matches_tgt(match_result.size());
			    for(uint32_t i = 0; i < match_result.size(); i++)
			    {
			        indices_matches_src[i] = match_result[i].queryIdx;
			        indices_matches_tgt[i] = match_result[i].trainIdx;
			    }
			    
			    // convert vector of keypoints to vector of points
			    vector<cv::Point2f> points_matches_src, points_matches_tgt;
			    cv::KeyPoint::convert(*keypoints_src_ptr, points_matches_src, indices_matches_src);
			    cv::KeyPoint::convert(*keypoints_tgt_ptr, points_matches_tgt, indices_matches_tgt);
			    
			    vector<uchar> mask_outliers;
			    findHomography(Mat(points_matches_src), Mat(points_matches_tgt), mask_outliers, CV_RANSAC, 3);
				
				cout << endl;
				
				for(int i = 0; i < mask_outliers.size(); i++)
				{
					cout << ((int) mask_outliers[i]) << " ";
					if(mask_outliers[i] == 1)
					{
						points_matches_tgt[i].y += 480;
					    cv::line(img_result, points_matches_src[i] , points_matches_tgt[i], cv::Scalar(255,0,0));
					    points_matches_tgt[i].y -= 480;
					}
				}				
				
				cout << endl;
				
				cv::imshow("test_result", img_result);
				cv::waitKey(0);
			}
			
			img_src = img_tgt;
			keypoints_src_ptr = keypoints_tgt_ptr;
			descriptors_src = descriptors_tgt;
			match_result.clear();
		}
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
	ros::init(argc, argv, "test_surf_match");
	ros::NodeHandle nh;
	
	Test_Surf_Match test_surf_match(nh);
	
	ros::spin();
	return 0;
}
