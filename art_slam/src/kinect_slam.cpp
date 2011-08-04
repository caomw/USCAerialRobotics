
#include <art_slam/kinect_slam.hpp>
#include <art_slam/hogman_cvt.hpp>
#include <pcl/common/transformation_from_correspondences.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace art_slam
{
	KinectSLAM::KinectSLAM(): optimizer(3, 2), num_count(0)
	{

	}
	
	KinectSLAM::~KinectSLAM()
	{
		for(uint i = 0; i < nodes.size(); i++) delete nodes[i];
	}
	
	void KinectSLAM::onInit()
	{
		nh = getMTNodeHandle();
		//initialize_gpu();
		pub_poses = nh.advertise<geometry_msgs::PoseArray>("slam/poses", 10);
		sub_kinect = nh.subscribe("/kinect/raw" , 10, &KinectSLAM::cb_sub_kinect, this);
	}

	void KinectSLAM::initialize_gpu()
	{
		ROS_INFO("GPU initializing ...");
		vector<cv::KeyPoint> vtemp1;
		vector<cv::DMatch> vtemp2;
		uchar data[100];
		for(int i = 0; i < 100; i++) data[i] = rand() % 256;
		//cv::gpu::GpuMat gpumat_temp(cv::Mat(10, 10, CV_8UC1, data));
		//detector(gpumat_temp, cv::gpu::GpuMat(), vtemp1);
		//matcher.match(gpumat_temp, gpumat_temp, vtemp2);
		ROS_INFO("GPU initialized!");
	}
	
	void KinectSLAM::cb_sub_kinect(const art_common::KinectMsg::ConstPtr msg)
	{
		num_count ++;
		if(num_count % 2 != 0) return;
		Node* node_new = new Node();
		cv::Mat image_temp(240, 320, CV_8UC1, (char*) &msg->image[0]);
		detector.detect(image_temp, node_new->keypoints);
		extractor.compute(image_temp, node_new->keypoints, node_new->descriptors);
		attach_depth_to_features(node_new, (short*) &(msg->depth[0]));
		if(nodes.size() == 0)
		{
			ASSERT_KEYPOINTS_NUMBER(node_new->keypoints);
			node_new->id = 0;
			nodes.push_back(node_new);
			/// TODO: analyze the groud surface and give the right initial orientation.
			optimizer.addVertex(node_new->id, Transformation3(), 1e9 * Matrix6::eye(1.0));
		}
		else
		{
			Node* node_old[4];
			Eigen::Matrix4f trafo[4];
			vector<cv::DMatch> matches[4], inliers[4];
			double error[4];
		
			for(int i = 1; i < 5 && i <= nodes.size(); i++)
			{
				int k = i - 1;
				node_old[k] = nodes.at(nodes.size() - i);	
				ASSERT_KEYPOINTS_NUMBER(node_new->keypoints);
				match_node_descriptors(node_new, node_old[k], matches[k]);
				ASSERT_MATCHING_NUMBER(matches[k]);
				get_inliers_ransac(node_new, node_old[k], trafo[k], matches[k], inliers[k], error[k]);
				cout << endl << "node " << "match " << inliers[k].size() << endl;
			}

			bool new_node_added = false;
			for(int i = 0; i < 4; i++)
			{
				if(inliers[i].size() > 10)
				{
					if(!new_node_added)
					{
						node_new->id = nodes.size();
						nodes.push_back(node_new);
						optimizer.addVertex(node_new->id, Transformation3(), 1e9 * Matrix6::eye(1.0));
						new_node_added = true;
					}
					optimizer.addEdge(optimizer.vertex(node_new->id), optimizer.vertex(node_old[i]->id), cvt_eigen_to_hogman(trafo[i]), Matrix6::eye(pow(inliers[i].size(), 2)));					
				}
			}

			optimizer.optimize(10, true);
			/// Publish the pose array.
			publish_pose_to_visualization(); 
		}
	}
	
	void KinectSLAM::attach_depth_to_features(Node* node, short* depth_data)
	{
		vector<cv::KeyPoint> keypoints_new;
		cv::Mat descriptors_new(0, node->descriptors.cols, node->descriptors.type());
		
		float centerX = 159.5, centerY = 119.5;
		float constant = 0.001 / 262.5;
		for(int i = 0; i < node->keypoints.size(); i++)
		{
			cv::Point2f* pt = &(node->keypoints[i].pt);
			float Z = static_cast<float>(depth_data[static_cast<int>(pt->y) * 320 + static_cast<int>(pt->x)]);
			if(!(isnan(Z) || Z == 0.0))
			{
				keypoints_new.push_back(node->keypoints[i]);
				descriptors_new.push_back(node->descriptors.row(i).clone());
				node->points.push_back(Eigen::Vector4f((pt->x - centerX) * Z * constant, (pt->y - centerY) * Z * constant, Z * 0.001, 0));
			}
		}
		node->keypoints = keypoints_new;
		node->descriptors = descriptors_new;
	}

	void KinectSLAM::match_node_descriptors(Node* node_new, Node* node_old, vector<cv::DMatch>& matches)
	{
		cv::gpu::GpuMat gpumat_new(node_new->descriptors), gpumat_old(node_old->descriptors);
		matcher.match(gpumat_new, gpumat_old, matches);
	}
	
	void KinectSLAM::get_inliers_ransac(Node* node_new, Node* node_old, Eigen::Matrix4f& trafo, vector<cv::DMatch>& matches, vector<cv::DMatch>& inliers, double& error)
	{
		/// Initialize variables and constants.
		uint min_inlier_threshold = matches.size() * 0.2;
		static double max_error = 0.04;
		static double max_error_suqared = max_error * max_error; /// make this constant so easy for compile.
		error = 1e6;
		Eigen::Matrix4f temp_trafo = Eigen::Matrix4f::Identity();
		srand((long) std::clock());
		
		/// Ransac iterations.
		for(int ransac_iteration = 0; ransac_iteration < 1000; ransac_iteration ++)
		{
			std::set<cv::DMatch> sample_matches;
			while(sample_matches.size() < 3)
				sample_matches.insert(matches.at(rand() % matches.size()));
			
			/// Calculate the trafo for samples.
			{
				pcl::TransformationFromCorrespondences tfc;
				for(set<cv::DMatch>::iterator it = sample_matches.begin(); it != sample_matches.end(); it ++)
					tfc.add(node_new->points.at(it->queryIdx).head<3>(), node_old->points.at(it->trainIdx).head<3>());
				temp_trafo = tfc.getTransformation().matrix();
			}
			/// Verify if this model is a good one.
			if(temp_trafo != temp_trafo) continue;
			double temp_error = 0;
			for(set<cv::DMatch>::iterator it = sample_matches.begin(); it != sample_matches.end(); it ++)
			{
				Eigen::Vector4f vec = temp_trafo * node_new->points.at(it->queryIdx) - node_old->points.at(it->trainIdx);
				double temp_this_error = vec.dot(vec);
				if(temp_this_error > max_error_suqared) continue;
				temp_error += sqrt(error);
			}
			if(temp_error > 10) continue;
			
			/// Apply the sample trafo to all points in the set; get gross error and number of inliers.
			vector<cv::DMatch> temp_inliers;
			temp_inliers.reserve(500);
			temp_error = 0;
			for(uint i = 0; i < matches.size(); i++)
			{
				Eigen::Vector4f vec = temp_trafo * node_new->points.at(matches[i].queryIdx) - node_old->points.at(matches[i].trainIdx);
				double temp_this_error = vec.dot(vec);
				if(temp_this_error > max_error_suqared) continue;
				temp_error += sqrt(error);
				temp_inliers.push_back(matches[i]);
			}
			temp_error /= temp_inliers.size();
			
			/// Compare the error and number of inliers to the best one we have got so far.
			if(temp_inliers.size() < min_inlier_threshold) continue;
			if(temp_error < error) { trafo = temp_trafo; inliers = temp_inliers; error = temp_error; }
			
			/// Speed up: if enough number of inliers have achieved, speed up the RANSAC process.
			if(inliers.size() > matches.size() * 0.5) ransac_iteration ++;
			if(inliers.size() > matches.size() * 0.8) ransac_iteration ++;
			
			/// Calculate the trafo for all inliers.
			{
				pcl::TransformationFromCorrespondences tfc;
				for(vector<cv::DMatch>::iterator it = inliers.begin(); it != inliers.end(); it ++)
					tfc.add(node_new->points.at(it->queryIdx).head<3>(), node_old->points.at(it->trainIdx).head<3>());
				temp_trafo = tfc.getTransformation().matrix();
			}

			if(temp_trafo != temp_trafo) continue;
			
			/// Apply the sample trafo to all points in the inliers; get gross error and number of inliers.
			temp_error = 0;
			temp_inliers.resize(0);
			temp_inliers.reserve(500);
			for(uint i = 0; i < matches.size(); i++)
			{
				Eigen::Vector4f vec = (temp_trafo * node_new->points[matches[i].queryIdx]) - node_old->points[matches[i].trainIdx];
				double temp_this_error = vec.dot(vec);
				if(temp_this_error > max_error_suqared) continue;
				temp_error += sqrt(error);
				temp_inliers.push_back(matches[i]);
			}
			temp_error /= temp_inliers.size();
			
			/// Compare the error and number of inliers to the best one we have got so far.
			if(temp_inliers.size() < min_inlier_threshold) continue;
			if(temp_error < error) { trafo = temp_trafo; inliers = temp_inliers; error = temp_error; }
		}
	}

	void KinectSLAM::publish_pose_to_visualization()
	{
		uint poses_size = optimizer.vertices().size();
		geometry_msgs::PoseArray poses;
		poses.poses.resize(poses_size);
		for(uint i = 0; i < poses_size; i++)
		{
			geometry_msgs::Pose& pose = poses.poses[i];
			_Vector<3, double> t = optimizer.vertex(i)->transformation.translation();
			_Vector<4, double> r = optimizer.vertex(i)->transformation.rotation();
			pose.position.x = t.x();
			pose.position.y = t.y();
			pose.position.z = t.z();
			pose.orientation.x = r.x();
			pose.orientation.y = r.y();
			pose.orientation.z = r.z();
			pose.orientation.w = r.w();
		}
		poses.header.frame_id = "/hahaframe";
		pub_poses.publish(poses);
	}
}

PLUGINLIB_DECLARE_CLASS(art_slam, kinect_slam, art_slam::KinectSLAM, nodelet::Nodelet);
