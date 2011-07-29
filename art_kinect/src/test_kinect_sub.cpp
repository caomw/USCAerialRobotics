#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <art_kinect/KinectMsg.h>

ros::Publisher pub_image, pub_depth;

sensor_msgs::Image msg_image, msg_depth;
cv::Mat mat_image, mat_depth;

void kinect_cb(const art_kinect::KinectMsg::ConstPtr& msg)
{
	cv::Mat mat_buf_image(1, msg->image.data.size(), CV_8UC1, (char*) &msg->image.data[0]);
	cv::Mat mat_buf_depth(1, msg->depth.data.size(), CV_8UC1, (char*) &msg->depth.data[0]);
	mat_image = cv::imdecode(mat_buf_image, -1);
	mat_depth = cv::imdecode(mat_buf_depth, -1);
	
	memcpy(&msg_image.data[0], (const char*) &mat_image.data[0], 76800);
	memcpy(&msg_depth.data[0], (const char*) &mat_depth.data[0], 153600);
	pub_image.publish(msg_image);
	pub_depth.publish(msg_depth);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "art_kinect_test_sub_node");
	ros::NodeHandle nh;

	mat_image.create(240, 320, CV_8UC1);
	mat_depth.create(240, 320, CV_16UC1);
	msg_image.encoding = sensor_msgs::image_encodings::MONO8;
	msg_depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	msg_image.width = msg_depth.width = 320; msg_image.height = msg_depth.height = 240;
	msg_image.step = 320; msg_depth.step = 640;
	msg_image.data.resize(76800); msg_depth.data.resize(153600);
	pub_image = nh.advertise<sensor_msgs::Image> ("/kinect/image", 10);
	pub_depth = nh.advertise<sensor_msgs::Image> ("/kinect/depth", 10);
	ros::Subscriber sub = nh.subscribe("/kinect_msg", 20, kinect_cb);
	
	ros::spin();
	return 0;
}
