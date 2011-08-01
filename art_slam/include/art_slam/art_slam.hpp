#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <geometry_msgs/Pose.h>

using namespace std;
namespace AIS = AISNavigation;

namespace art_slam
{	
	class KinectSLAM
	{
		ros::NodeHandle nh;

		ros::Subscriber sub_kinect;
		
		gpu::SURF_GPU detector;
		cv::BriefDescritporExtractor extractor;
		cv::gpu::BruteForceMatcher_GPU<Hamming> matcher;
		AISNavigation::HCholOptimizer3D optimizer;
		
		struct Node
		{
			int id;
			ros::Time stamp;
			vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;
			Eigen::Matrix<float, 3, eigen::Dynamic> points;
		};

		ros::Publisher pub_poses;
		
		vector<Node*> nodes;
		
		virtual void onInit();
		
		void cb_sync_subs(const sensor_msgs::Image::ConstPtr, const sensor_msgs::ImageConstPtr);
		
		void attach_depth_to_features(Node*, sensor_msgs::Image::ConstPtr);
		
		void RANSAC_3d_matching(Node*, Node*, Eigen::Matrix4f&, vector<cv::DMatch>&, double&);

		void initialize_gpu();

		void add_pose_to_visualization(Eigen::Matrix4f&);
		
	  public:

		KinectSLAM();
	  
		~KinectSLAM();
	}
}
