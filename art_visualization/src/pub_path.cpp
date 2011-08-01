#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace art_visualization
{
	class PubPath()
	{
		


	}
}

PLUGINLIB_DECLARE_CLASS(art_kinect, kinect_pub, art_kinect::KinectPub, nodelet::Nodelet);
