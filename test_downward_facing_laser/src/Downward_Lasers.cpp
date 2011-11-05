
#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;

typedef cv::Mat Mat;

using cv::Mat;

class Downward_Lasers
{	
	ros::NodeHandle nh;
	ros::Subscriber sub_image;
	uint sub_image_count;
	Mat img_color, img_gray;
	vector<cv::Vec3f> circles;
	
	
  public:

	Downward_Lasers(ros::NodeHandle& _nh): nh(_nh), sub_image_count(0), img_color(cv::Size(640, 480), CV_8UC3), img_gray(cv::Size(640, 480), CV_8UC3)
	{
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

			//convert to greyscale and smooth to avoid multiple flase circles in close proximity
      cv::cvtColor(img_color, img_gray, CV_BGR2GREY);
      cv::GaussianBlur( img_gray, img_gray, cv::Size(9, 9), 2, 2);

			//get circles
      cv::HoughCircles(img_gray, circles, CV_HOUGH_GRADIENT, 2, img_grey->rows/4, 100, 100, 0, 4 );
			
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
	ros::init(argc, argv, "test_feature_matching");
	ros::NodeHandle nh;
	
	Feature_Matching feature_matching(nh);
	
	ros::spin();
	return 0;
}
