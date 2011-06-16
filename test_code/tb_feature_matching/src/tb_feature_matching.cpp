#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann.hpp>

#include <art_common/timer.hpp>

using namespace std;

class TB_Feature_Matching
{	
	ros::NodeHandle nh;
	ros::Subscriber sub_image;
	uint sub_image_count;
	cv::Mat img_src, img_tgt;
	cv::SurfFeatureDetector surf_feature_detector;
	cv::SurfDescriptorExtractor surf_descriptor_extractor;
	
	boost::shared_ptr< vector<cv::KeyPoint> > keypoints_src_ptr, keypoints_tgt_ptr;
	
	cv::Mat descriptors_src, descriptors_tgt;
	
  public:

	TB_Feature_Matching(ros::NodeHandle& _nh): nh(_nh), sub_image_count(0),
		img_src(cv::Size(640, 480), CV_8UC3), img_tgt(cv::Size(640, 480), CV_8UC3)
	{
		sub_image = nh.subscribe("/usb_cam/image_raw", 10, &TB_Feature_Matching::sub_image_callback, this);		
	}
	
	~TB_Feature_Matching()
	{
		img_src.release();
		img_tgt.release();
		keypoints_src_ptr.reset();
		keypoints_tgt_ptr.reset();
	}
	
	void sub_image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		if(sub_image_count == 10 || sub_image_count == 20)
		{
			art::Timer t;
			
			t.reset();
			convert_image(msg, img_tgt);
			t.echospan("Convert Image");

			keypoints_tgt_ptr.reset(new vector<cv::KeyPoint>);
			
			t.reset();
			surf_feature_detector.detect(img_tgt, *keypoints_tgt_ptr);
			t.echospan("Detect Features");
			
			t.reset();
			surf_descriptor_extractor.compute(img_tgt, *keypoints_tgt_ptr, descriptors_tgt);
			t.echospan("Extract Features");
			
			// isn't the detect and extract part problematic?
			
			if(sub_image_count == 20)
			{	
				t.reset();
				cv::flann::Index flannindex_tgt(descriptors_tgt, cv::flann::KDTreeIndexParams(4));
				t.echospan("Build KdTree");
				
				int k = 1;
				
				cv::Mat indices(descriptors_src.rows, k, CV_32S);
				cv::Mat dists(descriptors_src.rows, k, CV_32F);
				
				t.reset();
				flannindex_tgt.knnSearch(descriptors_src, indices, dists, k, cv::flann::SearchParams(64));
				t.echospan("Search Matching in KdTree");
				
				cout << endl << "PROGRAM TERMINATED" << endl;
			}
			
			img_src = img_tgt;
			keypoints_src_ptr = keypoints_tgt_ptr;
			descriptors_src = descriptors_tgt;
		}
		
		++ sub_image_count;
	}

  private:
  
	void convert_image(const sensor_msgs::ImageConstPtr& _msg, cv::Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		_img = cv_image_ptr->image;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "programming_standard_TB_Feature_Matching");
	ros::NodeHandle nh;
	
	TB_Feature_Matching tb_feature_matching(nh);

	ros::Rate loop_rate(10);
	ros::spin();	
	return 0;
}
