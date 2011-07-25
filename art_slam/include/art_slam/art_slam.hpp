#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_hchol.h>

using namespace std;

namespace art_slam
{	
	class KinectSLAM
	{
		ros::NodeHandle nh;
		
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
		message_filters::Synchronizer<SyncPolicy> sub_sync;
		message_filters::Subscriber<Image> sub_image_gray, sub_depth_raw; 
		
		gpu::SURF_GPU detector;
		cv::BriefDescritporExtractor extractor;
		cv::gpu::BruteForceMatcher_GPU<Hamming> matcher;
		pcl::TransformationFromCorrespondences tfc;
		
		AISNavigation::HCholOptimizer3D optimizer;
		
		struct Node
		{
			vector<cv::KeyPoint> keypoints;
			cv::gpu::GpuMat descriptors;
			Eigen::Matrix<float, 3, eigen::Dynamic> points;
		};
		
		vector<Node*> list_node;
		
		virtual void onInit();
		
		void cb_sync_subs(const sensor_msgs::Image::ConstPtr, const sensor_msgs::ImageConstPtr);
		
		void attach_depth_to_features(Node*, sensor_msgs::Image::ConstPtr)
		
		void RANSAC_3d_matching(Node*, Node*, Eigen::Matrix4f&, vector<cv::DMatch>&, double&)
	  
	  public:
	  
		~KinectSLAM()
	}
}
