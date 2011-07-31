#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>


float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
ros::Publisher border_pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing

  sensor_msgs::PointCloud2 output;
  std::cout << "Got Some Point Cloud Data" << std::endl;

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromROSMsg (*input, point_cloud);


  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

  // -------------------------
  // -----Extract borders-----
  // -------------------------
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);

  // ----------------------------------
  // -----Show points in 3D viewer-----
  // ----------------------------------
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>::Ptr veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>::Ptr shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);

  //pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr;
  //pcl::PointCloud<pcl::PointWithRange>& veil_points   = *veil_points_ptr;
  //pcl::PointCloud<pcl::PointWithRange>& shadow_points = *shadow_points_ptr;

  sensor_msgs::PointCloud border_points;
  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
      {
        geometry_msgs::Point32 p;
        p.x = range_image.points[y*range_image.width + x].x;
        p.y = range_image.points[y*range_image.width + x].y;
        p.z = range_image.points[y*range_image.width + x].z;
        border_points.points.push_back (p);
      }

      //if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
      //  veil_points.points.push_back (range_image.points[y*range_image.width + x]);
      //if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
      //  shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
    }
  }


  sensor_msgs::PointCloud2 border_points_msg;
  sensor_msgs::convertPointCloudToPointCloud2(border_points, border_points_msg);
  border_pub.publish(border_points_msg);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "art_window");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("points", 1, cloud_cb);
  border_pub = nh.advertise<sensor_msgs::PointCloud2>("border_points", 1);

  // Spin
  ros::spin ();
}

