#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <art_kinect/KinectMsg.h>

using namespace std;

namespace art_kinect
{
	class KinectSub : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;

		cv::Mat mat_image, mat_depth;

	  public:
		KinectSub()
		{
			mat_image.create(240, 320, CV_8UC1);
			mat_depth.create(240, 320, CV_16UC1);
		}

	  private:  
		virtual void onInit()
		{
			nh = getMTNodeHandle();
			pub = nh.advertise<art_kinect::KinectMsg>("/kinect/raw", 10);
			sub = nh.subscribe("/kinect/compressed", 10, &KinectSub::cb_sub, this);
		}

		void cb_sub(const art_kinect::KinectMsg::ConstPtr& msg_compressed)
		{
			art_kinect::KinectMsg::Ptr msg_raw = boost::make_shared<art_kinect::KinectMsg>();
			msg_raw->id = msg_compressed->id;
			msg_raw->stamp = msg_compressed->stamp;
			msg_raw->image.resize(76800); msg_raw->depth.resize(153600);

			cv::Mat mat_buf_image(1, msg_compressed->image.size(), CV_8UC1, (char*) &msg_compressed->image[0]);
			cv::Mat mat_buf_depth(1, msg_compressed->depth.size(), CV_8UC1, (char*) &msg_compressed->depth[0]);
			mat_image = cv::imdecode(mat_buf_image, -1);
			mat_depth = cv::imdecode(mat_buf_depth, -1);

			/// TODO: Fool OpenCV.
			memcpy(&msg_raw->image[0], (const char*) &mat_image.data[0], 76800);
			memcpy(&msg_raw->depth[0], (const char*) &mat_depth.data[0], 153600);
			pub.publish(msg_raw);
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_kinect, kinect_sub, art_kinect::KinectSub, nodelet::Nodelet);
