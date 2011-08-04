#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <openni_camera/openni_device_kinect.h>
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_image.h>
#include <openni_camera/openni_depth_image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <art_common/KinectMsg.h>

using namespace std;
using namespace openni_wrapper;

namespace art_kinect
{
	struct StampedImage
	{
		struct { ros::Time stamp; } header;
		boost::shared_ptr<Image> image;
	};
	
	struct StampedDepth
	{
		struct { ros::Time stamp; } header;
		boost::shared_ptr<DepthImage> depth;
	};
}

namespace ros
{
	namespace message_traits
	{
		template<>
		struct TimeStamp<art_kinect::StampedImage>
		{
			static ros::Time value(const art_kinect::StampedImage& msg)
			{
				return msg.header.stamp;
			}
		};

		template<>
		struct TimeStamp<art_kinect::StampedDepth>
		{
			static ros::Time value(const art_kinect::StampedDepth& msg)
			{
				return msg.header.stamp;
			}
		};
	}
}

namespace art_kinect
{
	class KinectPub : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		boost::shared_ptr<openni_wrapper::OpenNIDevice> device;
		typedef message_filters::sync_policies::ApproximateTime<StampedImage, StampedDepth> SyncPolicy;
		message_filters::Synchronizer<SyncPolicy> synchronizer;
		ros::Publisher pub;
		art_common::KinectMsg msg_pub;
		cv::Mat mat_image, mat_depth;
		string encode_format;
		vector<int> encode_params;
		uint8_t count_skip;

	  public:
		KinectPub(): synchronizer(SyncPolicy(10)), encode_format(".png")
		{
			/// Configure variables.
			mat_image.create(240, 320, CV_8UC1);
			mat_depth.create(240, 320, CV_16UC1);
			encode_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			encode_params.push_back(0);
			msg_pub.id = -1;
			msg_pub.image.reserve(76800);
			msg_pub.depth.reserve(153600);
			count_skip = 0;
		}

	  private:
		virtual void onInit()
		{
			/// Configure publisher and synchronizer.
			nh = getMTNodeHandle();
			pub = nh.advertise<art_common::KinectMsg>("/kinect/compressed", 10);
			synchronizer.registerCallback(&KinectPub::callback_synchronizer, this);
			
			/// Load and configure the Kinect device.
			OpenNIDriver& driver = OpenNIDriver::getInstance();
			driver.updateDeviceList();
			device = driver.getDeviceByIndex(0);
			XnMapOutputMode output_mode;
			output_mode.nXRes = XN_VGA_X_RES;
			output_mode.nYRes = XN_VGA_Y_RES;
			output_mode.nFPS = 30;
			device->setDepthOutputMode(output_mode);
			device->setImageOutputMode(output_mode);
			device->setDepthRegistration(true);
			dynamic_cast<DeviceKinect*>(device.get())->setDebayeringMethod(ImageBayerGRBG::EdgeAwareWeighted);

			/// Register callbacks and start the streams.
			device->registerImageCallback(&KinectPub::cb_image, *this);
			device->registerDepthCallback(&KinectPub::cb_depth, *this);
			device->startImageStream();
			device->startDepthStream();
		}

		void cb_image(boost::shared_ptr<Image> image, void* cookie)
		{
			ros::Time time_now = ros::Time::now();
			boost::shared_ptr<StampedImage> msg_image = boost::make_shared<StampedImage>();
			msg_image->header.stamp = time_now;
			msg_image->image = image;
			synchronizer.add<0>(msg_image);
		}
		
		void cb_depth(boost::shared_ptr<DepthImage> depth, void* cookie)
		{
			ros::Time time_now = ros::Time::now();
			boost::shared_ptr<StampedDepth> msg_depth = boost::make_shared<StampedDepth>();
			msg_depth->header.stamp = time_now;
			msg_depth->depth = depth;
			synchronizer.add<1>(msg_depth);
		}
		
		void callback_synchronizer(const boost::shared_ptr<StampedImage> msg_image, const boost::shared_ptr<StampedDepth> msg_depth)
		{
			if(count_skip == 2)
			{
				msg_pub.id ++;
				count_skip = 0;
				msg_pub.stamp = msg_image->header.stamp; /// TODO
				msg_image->image->fillGrayscale(320, 240, &mat_image.data[0], 320);
				msg_depth->depth->fillDepthImageRaw(320, 240, (short*) &mat_depth.data[0], 640);
				cv::imencode(encode_format, mat_image, msg_pub.image, encode_params);
				cv::imencode(encode_format, mat_depth, msg_pub.depth, encode_params);
				pub.publish(msg_pub);
				count_skip == 0;
			}
			count_skip ++;
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_kinect, kinect_pub, art_kinect::KinectPub, nodelet::Nodelet);
