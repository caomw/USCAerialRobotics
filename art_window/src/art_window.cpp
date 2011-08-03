#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <art_kinect/KinectMsg.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

using namespace std;
using namespace pcl;

namespace art_window
{	
	class WindowLocalization : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		ros::Subscriber sub_kinect;
		ros::Publisher pub_cloud, pub_model, pub_model_aligned;
		PointCloud<PointXYZ>::Ptr model;
		/// tuning stuff ---
		int threshold1;
		int threshold2;
		/// end of tuning stuff ---

	  public:
		WindowLocalization(): model(new PointCloud<PointXYZ>()) {}

	  private:
		virtual void onInit()
		{
			nh = getMTNodeHandle();
			pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/window/points", 10);
			//pub_model = nh.advertise<sensor_msgs::PointCloud2>("/window/model", 10);
			pub_model_aligned = nh.advertise<sensor_msgs::PointCloud2>("/window/model_aligned", 10);
			//create_window_model(1.0, 1.0, 40);
			sub_kinect = nh.subscribe("/kinect/raw", 10, &WindowLocalization::cb_sub_kinect, this);
			threshold1 = 100;
			threshold2 = 200;
		}

		void cb_sub_kinect(const art_kinect::KinectMsg::ConstPtr input)
		{
			cv::Mat img;
			img.create(240, 320, CV_8UC1);
			short* data_ptr = (short*)(&input->depth[0]);
			for(int i = 0; i <= 76800; i++)
			{
				if(data_ptr[i] < 4096)
					img.data[i] = (uchar) (data_ptr[i] / 16);
				else
					img.data[i] = 0;
			}
			
			cv::Mat morphImg;
			cv::morphologyEx(img, morphImg, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 5);
			cv::Mat edges;
			cv::Canny(morphImg, edges, threshold1, threshold2);
			PointCloud<PointXYZ>::Ptr cloud_original(new PointCloud<PointXYZ>()), cloud(new PointCloud<PointXYZ>());
			
			float centerX = 159.5, centerY = 119.5;
			float constant = 1.0 / 262.5;
			for(int i = 0; i < 76800; i++)
			{
				if(edges.data[i] == 255)
				{
					float Z = data_ptr[i] * 0.001;
					if(!isnan(Z))
					{
						PointXYZ newpoint;
						newpoint.x = ((i % 320) - centerX) * Z * constant;
						newpoint.y = ((i / 320) - centerY) * Z * constant;
						newpoint.z = Z;
						cloud_original->push_back(newpoint);
					}
				}
			}

			StatisticalOutlierRemoval<PointXYZ> sor;
			sor.setInputCloud(cloud_original);
			sor.setMeanK(10);
			sor.setStddevMulThresh (1);
			sor.filter(*cloud);
			
			sensor_msgs::PointCloud2 msg_cloud;
			toROSMsg(*cloud, msg_cloud);
			msg_cloud.header.frame_id = "/haha1";
			pub_cloud.publish(msg_cloud);

			/*IterativeClosestPointNonLinear<PointXYZ, PointXYZ> icp;		
			icp.setInputCloud(model);
			icp.setInputTarget(cloud);
			PointCloud<PointXYZ> model_aligned;
			icp.align(model_aligned);
			sensor_msgs::PointCloud2 msg_cloud2;
			toROSMsg(model_aligned, msg_cloud2);
			msg_cloud2.header.frame_id = "/haha2";
			pub_model_aligned.publish(msg_cloud2);*/
		}

		void create_window_model(float width, float height, int num_points_side)
		{
			float unit_length = 2.0 / num_points_side;
			width *= 0.5; height *= 0.5;
			model->points.resize(num_points_side *4);
			PointXYZ* points_ptr = &model->points[0];
			for(int i = 0; i < num_points_side; i++, points_ptr++)
			{
				points_ptr->z = 0; points_ptr->y = width * (unit_length * i - 1.0); points_ptr->x = height;
			}
			for(int i = 0; i < num_points_side; i++, points_ptr++)
			{
				points_ptr->z = 0; points_ptr->y = width; points_ptr->x = height * (unit_length * i - 1.0);
			}
			for(int i = 0; i < num_points_side; i++, points_ptr++)
			{
				points_ptr->z = 0; points_ptr->y = width * (unit_length * i - 1.0); points_ptr->x = -height;
			}
			for(int i = 0; i < num_points_side; i++, points_ptr++)
			{
				points_ptr->z = 0; points_ptr->y = -width; points_ptr->x = height * (unit_length * i - 1.0);
			}
			/*sensor_msgs::PointCloud2 msg;
			toROSMsg(*model, msg);
			msg.header.frame_id="/hahaframe";
			pub_model.publish(msg);*/
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_window, window_localization, art_window::WindowLocalization, nodelet::Nodelet);
