#include <art_slam/kinect_slam.hpp>

namespace art_slam
{
	KinectSLAM::KinectSLAM(): optimizer(3, 2)
	{
		initialize_gpu();
	}
	
	KinectSLAM::~KinectSLAM()
	{
		for(int i = 0; i < nodes.size(); i++) delete nodes[i];
	}
	
	virtual void KinectSLAM::onInit()
	{
		nh = getMTNodeHandle();
		pub_poses = nh.advertise<geometry_msgs::PoseArray>("slam/poses", 10);
		sub_kinect = nh.subscribe("/kinect/raw" , 10, cb_sub_kinect);
	}

	void initialize_gpu()
	{
		cv::gpu::GpuMat gpumat_temp(2, 2, CV_8UC1);
		detector(gpumat_temp, cv::gpu::GpuMat(), vector<cv::Keypoint>());
		matcher.match(gpumat_temp, gpumat_temp, vector<cv::DMatch>());
	}
	
	void KinectSLAM::cb_sub_kinect(const art_kinect::KinectMsg::ConstPtr msg)
	{
		Node* node_new = new Node();
		detector(cv::Mat(240, 320, CV_8UC1, msg->image), gpu::GpuMat(), node->keypoints);
		attach_depth_to_features(node_new, msg->depth);
		extractor.compute(image_temp, node_new->keypoints, descriptors);
				
		if(list_node.size() == 0)
		{
			node_new->id = 0;
			nodes.push_back(node_new);
			/// TODO: analyze the groud surface and give the right initial orientation.
			optimizer.addVertex(node_new->id, Transformation3(), 1e9 * Matrix6::eye(1.0));
		}
		else
		{
			Node* node_old = nodes.back();
			/// Start RANSAC matching.
			Eigen::Matrix4f trafo;
			vector<cv::DMatch> inliers;
			double error;
			match_node_descriptors(node_new, node_old, inliers);			
			get_inliers_ransac(node_new, node_old, trafo, inliers, error);
			
			/// If it's good node, add it to graph system, and then optimize the graph.
			node_new->id = nodes.size();
			nodes.push_back(node_new);
			optimizer.addVertex(node_new->id, Transformation3(), 1e9 * Matrix6::eye(1.0));
			optimizer.addEdge(optimizer->vertex(node_old->id), optimizer->vertex(node_new->id), cvt_eigen_to_hogman(trafo), Matrix6::eye(pow(inliers.size(), 2)));
			optimizer.optimize(10, true);

			/// Publish the pose array.
			publish_poses_to_visualization(); 
		}
	}
	
	void KinectSLAM::attach_depth_to_features(Node* node, sensor_msgs::Image::ConstPtr msgs_depth)
	{
		vector<int> list_delete;
		float centerX = 320.0 - 0.5f, centerY = 240.0 - 0.5f;
		float constant = 0.001 / device_->getDepthFocalLength (depth_width_);
		for(int i = 0; i < node->keypoints.size(); i++)
		{
			cv::Point2f* pt = &(node->keypoints[i].pt);
			if(pt->x != pt->x || pt->y != pt->y)
				list_delete.push_back(i);
			else
			{
				node->points(0, i) = (pt.x - centerX) * msgs_depth->data[pt.y * 320 + pt.x] * constant;
				node->points(1, i) = (pt.y - centerY) * msgs_depth->data[pt.y * 320 + pt.x] * constant;
				node->points(2, i) = msgs_depth->data[pt.y * 320 + pt.x] * 0.001;
			}
		}
		int last_valid_pos = node->keypoints.size() - 1;
		for(int i = list_delete.size() - 1; i >= 0; i--)
		{
			if(list_delete[i] < last_valid_pos)
			{
				node->keypoints[i] = node->keypoints[last_valid_pos];
				node->points.col(i) = node->points.col(last_valid_pos);
				last_valid_pos --;
			}
			else if(list_delete[i] == last_valid_pos) last_valid_pos --;
		}
		int newsize = node->keypoints.size() - list_delete.size();
		node->keypoints.resize(newsize);
		node->points.conservativeResize(Eigen::NoChange, newsize);
	}

	void KinectSLAM::match_node_descriptors(Node* node_new, Node* node_old, vector<cv::DMatch>& matches)
	{
		cv::gpu::GpuMat gpumat_new(node_new->descriptors), gpumat_old(node_old->descriptors);
		matcher.match(gpumat_new, gpumat_old, matches);
	}
	
	void KinectSLAM::get_inliers_ransac(Node* node_new, Node* node_old, Eigen::Matrix4f& trafo, vector<cv::DMatch>& inliers, double& error)
	{
		/// Initialize variables and constants.
		int min_inlier_threshold = initial_matches->size() >> 1;
		static double max_error = xxxx; /// TODO
		static double max_error_suqare = max_error * max_error; /// make this constant so easy for compile.
		error = 1e6;
		inliers.resize(0);
		Eigen::Matrix4f temp_trafo = Eigen::Matrix4f::Identity();
		srand((long) std::clock());
		
		/// Ransac iterations.
		for(int ransac_iteration = 0; ransac_iteration < 1000; ransac_iteration ++)
		{
			std::set<cv::DMatch> sample_matches;
			while(sample_matches.size() < 4)
				sample_matches.insert(matches.at(rand() % initial_matches->size()));
			
			/// Calculate the trafo for samples.
			for(InputIterator it = sample_matches.begin(); it != iter_end; it ++)
			    tfc.add(node_new->points.col(it->queryIdx), node_old->points.col(it->trainIdx));
			temp_trafo = tfc.getTransformation().matrix();
			
			/// TODO: Verify if this model is a good one.
			
			/// Apply the sample trafo to all points in the set; get gross error and number of inliers.
			vector<cv::DMatch> temp_inliers;
			temp_inliers.reserve(1000);
			double temp_error = 0;
			for(int i = 0; i < matches.size(); i++)
			{
				Eigen::Vector4f vec = (temp_trafo * node_new->points.col(matches[i].queryIdx)) - node_old->points.col(matches[i].trainIdx);
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
			if(inlier.size() > initial_matches->size()*0.5) ransac_iteration ++;
			if(inlier.size() > initial_matches->size()*0.8) ransac_iteration ++;
			
			/// Calculate the trafo for all inliers.
			for(InputIterator it = inliers.begin(); it != inliers.end(); it ++)
			    tfc.add(node_new->points.col(it->queryIdx), node_old->points.col(it->trainIdx));
			temp_trafo = tfc.getTransformation().matrix();
			
			/// Apply the sample trafo to all points in the inliers; get gross error and number of inliers.
			temp_inliers_size = 0;
			temp_error = 0;
			temp_inliers.resize(0);
			temp_inliers.reserve(1000);
			for(int i = 0; i < matches.size(); i++)
			{
				Eigen::Vector4f vec = (temp_trafo * node_new->points.col(matches[i].queryIdx)) - node_old->points.col(matches[i].trainIdx);
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

	void publish_pose_to_visualization()
	{
		int poses_size = optimizer.vertices().size();
		geometry_msgs::PoseArray poses;
		poses.poses.resize(poses_size)
		for(int i = 0; i < poses_size; i++)
		{
			geometry_msgs::Pose& pose = poses.poses[i];
			_Vector<3, double>& t = optimizer.vertex(i)->transformation.translation();
			_Vector<4, double>& r = optimizer.vertex(i)->transformation.rotation();
			pose.point.x = t.x;
			pose.point.y = t.y;
			pose.point.z = t.z;
			pose.orientation.x = r.x;
			pose.orientation.y = r.y;
			pose.orientation.z = r.z;
			pose.orientation.w = r.w;
		}
		pub_poses.publish(poses);
	}
}

PLUGINLIB_DECLARE_CLASS(art_slam, kinect_slam, art_slam::KinectSLAM, nodelet::Nodelet);
