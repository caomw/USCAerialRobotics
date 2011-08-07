#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <art_common/KinectMsg.h>

using namespace std;

namespace art_visualization
{
	class PubPoints : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;

		virtual void onInit()
		{
			nh = getMTNodeHandle();
			pub = nh.advertise<sensor_msgs::PointCloud2>("/kinect/pointcloud", 10);
			sub = nh.subscribe("/kinect/raw", 10, &PubPoints::cb_sub, this);
		}

		void cb_sub(const art_common::KinectMsg::ConstPtr& msg)
		{
			pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
			pointcloud.header.stamp = msg->stamp;
			pointcloud.header.frame_id = "/hahaframe";
			pointcloud.is_dense = false;
			float centerX = 159.5, centerY = 119.5;
			float constant = 1.0 / 262.5;
		
			const char* rgb_buffer = reinterpret_cast<const char*>(&msg->image[0]);
			const short* depth_buffer = reinterpret_cast<const short*>(&msg->depth[0]);
			int idx = 0;
			for(int v = 0; v < 240; v++)
			{
				for (int u = 0; u < 320; u++, idx++)
				{
			        float Z = static_cast<float>(depth_buffer[idx]);

					if(!(isnan(Z) || Z == 0.0))
					{
						pointcloud.points.push_back(pcl::PointXYZRGB());
						Z *= 0.001;
						pointcloud.points.back().x = (u - centerX) * Z * constant;
						pointcloud.points.back().y = (v - centerY) * Z * constant;
						pointcloud.points.back().z = Z;
						union{ struct { unsigned char Blue, Green, Red, Alpha; }; float float_value; } color;
				        color.Red = rgb_buffer[idx];
				        color.Green = rgb_buffer[idx];
				        color.Blue = rgb_buffer[idx];
				        color.Alpha = 0;
				        pointcloud.points.back().rgb = color.float_value;
					}
				}
			}
			sensor_msgs::PointCloud2 haha;
			pcl::toROSMsg(pointcloud, haha);
			haha.header.frame_id = "/hahaframe";
			haha.header.stamp = ros::Time::now();
			pub.publish(haha);
			
			
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_visualization, pub_points, art_visualization::PubPoints, nodelet::Nodelet);
