#ifndef _ART_SLAM_KINECT_SLAM_
#define _ART_SLAM_KINECT_SLAM_

#define ASSERT_KEYPOINTS_NUMBER(KEYPOINTS) if(KEYPOINTS.size() < 50) { ROS_ERROR("Too few keypoints."); return; }
#define ASSERT_MATCHING_NUMBER(MATCHES) if(MATCHES.size() < 36) { ROS_ERROR("Too few matches."); return; }


#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <art_common/KinectMsg.h>
#include <Eigen/Dense>

using namespace std;
namespace AIS = AISNavigation;

namespace art_slam
{	
	class KinectSLAM : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		ros::Subscriber sub_kinect;
		ros::Publisher pub_poses;

		cv::SurfFeatureDetector detector;
		cv::BriefDescriptorExtractor extractor;
		
		cv::BruteForceMatcher<cv::HammingLUT> matcher;
		AIS::HCholOptimizer3D optimizer;

		uint num_count;
		
		struct Node
		{
			int id;
			ros::Time stamp;
			vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;
			vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > points;
		};
		
		vector<Node*> nodes;
		
		virtual void onInit();
		
		void cb_sub_kinect(const art_common::KinectMsg::ConstPtr);
		
		void attach_depth_to_features(Node*, short*);
		
		void match_node_descriptors(Node*, Node*, vector<cv::DMatch>&);
	
		void get_inliers_ransac(Node*, Node*, Eigen::Matrix4f&, vector<cv::DMatch>&, vector<cv::DMatch>&, double&);

		void initialize_gpu();

		void publish_pose_to_visualization();
		
	  public:

		KinectSLAM();
	  
		~KinectSLAM();
	};
}

#endif
