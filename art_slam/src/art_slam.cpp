namespace art_slam
{	
	virtual void KinectSLAM::onInit()
	{
		nh = getMTNodeHandle();
		sub_image_gray = message_filters::Subscriber<sensor_msgs::Image>(nh, "/kinect/image_gray" , 10);
		sub_depth_raw = message_filters::Subscriber<sensor_msgs::Image>(nh, "/kinect/depth_raw" , 10);
		sub_sync = message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), sub_image_gray, sub_depth_raw);
		sub_sync.registerCallback(boost::bind(&KinectSLAM::cb_sync_subs, this, _1, _2));
		
		optimizer = AISNavigation::HCholOptimizer3D(3, 2)
		
		initialize_gpu(); /// TODO
	}
	
	~KinectSLAM()
	{
		for(int i = 0; i < list_node.size(); i++) delete list_node[i];
	}
	
	void KinectSLAM::cb_sync_subs(const sensor_msgs::Image::ConstPtr msg_image, const sensor_msgs::Image::ConstPtr msg_depth)
	{
		/// Create new node.
		Node* node_new = new Node();
		
		/// Detect features and store them as keypoints.
		cv::Mat image_temp(240, 320, CV_8UC1, (char*) &(msg_image->data[0])); /// TODO: find out why 240 x 320
		detector(img_new, gpu::GpuMat(), node->keypoints);
		
		/// Get the 3D points of these features, filter out those without valid depth.
		attach_depth_to_features_impl(node_new, msg_depth);
		
		/// Extract feature descriptors.
		cv::Mat descriptors_temp;
		extractor.compute(image_temp, node_new->keypoints, descriptors_temp);
		node->descriptors.upload(descriptors_temp);
		
		if(list_node.size() > 0)
		{
			/// Start RANSAC matching.
			
			Eigen::Matrix4f trafo;
			vector<cv::DMatch> inliers;
			double error;
			RANSAC_3d_matching(node_new, list_node.back(), trafo, inliers, error);
			
			
						
			/// Figure out which previous nodes to match.
			
			/// Match those previous.
			
			/// Filter out big trafo and add to the graph system.
			
			/// Optimize the graph.
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
	
	void KinectSLAM RANSAC_3d_matching(Node* node_new, Node* node_old, Eigen::Matrix4f& trafo, vector<cv::DMatch>& inliers, double& error)
	{
		vector<cv::DMatch> matches;
		matcher.match(node_new->descriptors, list_node.back->descriptors, matches);
		
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
	
}
