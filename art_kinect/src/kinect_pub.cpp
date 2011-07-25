#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <openni_camera/openni_device_kinect.h>
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_image.h>
#include <openni_camera/openni_depth_image.h>
#include <sensor_msgs/Image.h>

namespace art_kinect
{
	class KinectPub : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		OpenNIDriver& driver;
		boost::shared_ptr<openni_wrapper::OpenNIDevice> device;
		ros::Publisher pub_image_gray, pub_depth_raw;
		
		virtual void onInit()
		{
			nh = getMTNodeHandle();
			driver = OpenNIDriver::getInstance();
			driver.updateDeviceList();
			device = driver.getDeviceByIndex(0);
			
			pub_image_gray = nh.advertise<sensor_msgs::Image>("/kinect/image_gray", 10);
			pub_depth_raw = nh.advertise<sensor_msgs::Image>("/kinect/depth_raw", 10);
			
			XnMapOutputMode output_mode;
			output_mode.nXRes = XN_VGA_X_RES;
			output_mode.nYRes = XN_VGA_Y_RES;
			output_mode.nFPS = 30;
			device->setDepthOutputMode(output_mode);
			device->setImageOutputMode(output_mode);
			device->setDepthRegistration(true);
			dynamic_cast<DeviceKinect*>(device.get())->setDebayeringMethod(ImageBayerGRBG::EdgeAwareWeighted);
			
			device->registerImageCallback(image_callback);
			device->registerDepthCallback(depth_callback);
			
			device->startImageStream();
			device->startDepthStream();
		}
		
		void image_callback(boost::shared_ptr<Image> image, void* cookie)
		{
			ros::Time time_now = ros::Time::now();
			sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
			msg->header.stamp = time_now;
			msg->data.resize(76800);
			image->fillGrayscale(320, 240, &gray_msg->data[0], 320);
			pub_image_gray.publish(msg);
		}
		
		void depth_callback(boost::shared_ptr<DepthImage> image, void* cookie)
		{
			ros::Time time_now = ros::Time::now();
			sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
			msg->header.stamp = time_now;
			msg->data.resize(153600);
			image->fillDepthImageRaw(320, 240, (short*) &depth_msg->data[0], 640);
			pub_depth_raw.publish(msg);
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_kinect, kinect_pub, art_kinect::KinectPub, nodelet::Nodelet);
