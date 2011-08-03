#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <art_kinect/KinectMsg.h>

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

		void cb_sub(const art_kinect::KinectMsg::ConstPtr& msg)
		{
			pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
			pointcloud.points.resize(76800);
			pointcloud.header.stamp = msg->stamp;
			pointcloud.header.frame_id = "/hahaframe";
			pointcloud.height = 240;
			pointcloud.width = 320;
			pointcloud.is_dense = false;
			float centerX = 159.5, centerY = 119.5;
			float constant = 1.0 / 262.5;
		
			const char* rgb_buffer = reinterpret_cast<const char*>(&msg->image[0]);
			const short* depth_buffer = reinterpret_cast<const short*>(&msg->depth[0]);
			int idx = 0;
			pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = pointcloud.points.begin();
			for(int v = 0; v < 240; v++)
			{
				for (int u = 0; u < 320; u++, idx++, pt_iter++)
				{
			        float Z = static_cast<float>(depth_buffer[idx]);
			        {
						if(!isnan(Z))
						{
							Z *= 0.001;
							pt_iter->x = (u - centerX) * Z * constant;
							pt_iter->y = (v - centerY) * Z * constant;
							pt_iter->z = Z;
						}
						else
						{
							pt_iter->x = pt_iter->y = pt_iter->z = Z;
						}
						
						union{ struct { unsigned char Blue, Green, Red, Alpha; }; float float_value; } color;
				        color.Red = rgb_buffer[idx];
				        color.Green = rgb_buffer[idx];
				        color.Blue = rgb_buffer[idx];
				        color.Alpha = 0;
				        pt_iter->rgb = color.float_value;
			        }
				}
			}
			sensor_msgs::PointCloud2 haha;
			pcl::toROSMsg(pointcloud, haha);
			haha.header.frame_id = "/hahaframe";
			pub.publish(haha);
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_visualization, pub_points, art_visualization::PubPoints, nodelet::Nodelet);
