#include <ros/ros.h>
#include <cmath>
#include <string>
#include <iostream>
#include <cv.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fovis/fovis.hpp>

//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>

//#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <string>


#include <ros/ros.h>
#include <cmath>
#include <string>
#include <iostream>
#include <cv.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>


namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace cv;
using namespace Eigen;

struct float3{ float x; float y; float z;};

class kinectNode {

public:

private:

    ros::NodeHandle nh;
    ros::Subscriber subImage;
    uint subImageCount;

    Mat imgColor;
    Mat imgDepth;
    Mat imgDepthPrevious;
    Mat imgGray;
    Mat imgGrayPrevious;
    Mat imgCorner;
    fovis::VisualOdometry* odom;
    fovis::CameraIntrinsicsParameters rgb_params_;

    fovis::DepthImage * depth_image_;
    fovis::Rectification * rect;
    int width;
    int height;
    //tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher odom_pub_;

public:

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> Policy_sync_subs;
    message_filters::Synchronizer<Policy_sync_subs> sync_subs;

    kinectNode(ros::NodeHandle& _nh):
	nh(_nh), sub_rgb(nh, "/camera/rgb/image_rect", 10), sub_depth(nh, "/camera/depth/image_rect", 10), 
    sync_subs(Policy_sync_subs(10), sub_rgb, sub_depth) {


	this->nh = _nh;

	//odom_pub_ = _nh.advertise<nav_msgs::Odometry>("fovis_odometry", 1000);

	subImageCount = 0;
	imgColor = Mat(cv::Size(640, 480), CV_8UC3);
	imgDepth = Mat(cv::Size(640, 480), CV_32FC1);
	imgDepthPrevious = Mat(cv::Size(640, 480), CV_32FC1);
	imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
	imgGray = Mat(cv::Size(640, 480), CV_8UC1);

	// Create some intrinsic parameters for the kinect.
	// TODO: Pull these from the param server or something
	memset(&rgb_params_, 0, sizeof(fovis::CameraIntrinsicsParameters));
	width = 640;
	height = 480;
	rgb_params_.width = width;
	rgb_params_.height = height;
	rgb_params_.fx = 528.49404721; 
	rgb_params_.fy = rgb_params_.fx;
	rgb_params_.cx = width / 2.0;
	rgb_params_.cy = height / 2.0;

	// get the RGB camera parameters of our device
	rect = new fovis::Rectification(rgb_params_);
	fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
	// If we wanted to play around with the different VO parameters, we could set
	// them here in the "options" variable.

	// setup the visual odometry
	odom = new fovis::VisualOdometry(rect, options);
	depth_image_ = new fovis::DepthImage(rgb_params_, width, height);

	//subscribe to the image
	//subImage = nh.subscribe("/camera/rgb/image_rect", 10, &kinectNode::subImageCallback, this);
	//subImage = nh.subscribe("/camera/depth/image_rect", 10, &kinectNode::subImageCallback, this);

	//Create a cv window to view image in for now
	sync_subs.registerCallback(boost::bind(&kinectNode::subImageCallback, this, _1, _2));		

	//cv::namedWindow("Image");
    }

    ~kinectNode() {
	//cv::destroyWindow("Image");
	imgColor.release();
	imgDepth.release();
	imgGray.release();
    }

    std::string  isometryToString(const Eigen::Isometry3d& m) {
	char result[80];
	memset(result, 0, sizeof(result));
	Eigen::Vector3d xyz = m.translation();
	Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
	snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", 
		 xyz(0), xyz(1), xyz(2), 
		 rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
	return std::string(result);
    }



private:

    void subImageCallback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth) {

	// std::cout << "GOT A NEW DEPTH IMAGE: " << msg_depth->encoding << " !!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	// std::cout << "Image Callback: " << msg_rgb->header.stamp << " | " << msg_depth->header.stamp << std::endl;

	subImageCount++;
	// cout << "Got image number: " << subImageCount << endl;
			
                        
	// convert image from ROS image message to OpenCV Mat
	// convert_rgb_image(msg, imgColor);
			
	convert_depth_image(msg_depth, imgDepth);
	convert_rgb_image(msg_rgb, imgColor);


	cv::imshow("rgb image", imgColor);
	cv::waitKey(1);

	cv::imshow("depth image", imgDepth);
	cv::waitKey(1);

	cvtColor(imgColor, imgGray, CV_RGB2GRAY);
			
	//Do things with the image here
	
	vector<Point2f> *corners = new vector<Point2f>();
	cvtColor(imgColor, imgGray, CV_RGB2GRAY);
	
	if(subImageCount <= 1) {
	    imgGrayPrevious = imgGray.clone();
	    imgDepthPrevious = imgDepth.clone();
	    return;
	}
	
	goodFeaturesToTrack(imgGrayPrevious, *corners, 100, .1, 10, Mat(), 3, true, .04);

	//corners->clear();
	// for(int i=50; i<320; i+=50)
	// {
	//     for(int j=100; j<320; j+=100)
	//     {
	// 	Point2f temp(i,j);
	// 	corners->push_back(temp);
	//     }
	// }

	
	vector<Point2f> *nextCorners = new vector<Point2f>((*corners).size(), Point2f(0, 0));
	vector<uchar> status;
	vector<float> errors;
	

	calcOpticalFlowPyrLK(imgGrayPrevious, imgGray, *corners, *nextCorners, status, errors, Size(21,21), OPTFLOW_USE_INITIAL_FLOW, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.3), 0, .0001);
	
	
	for(unsigned int i = 0; i < corners->size(); i++) {
	    circle(imgColor, (*corners)[i], 8, cvScalar(255, 0, 0), 2, 8, 0);
	    line(imgColor, (*corners)[i], (*nextCorners)[i], cvScalar(0, 0, 255), 1, 8, 0);
	}

	vector<float3> * flows = new vector<float3>;
	vector<float3> * corners3d = new vector<float3>;
	vector<float3> * nextCorners3d = new vector<float3>;

	corners3d->clear();
	nextCorners3d->clear();
	flows->clear();

	for(int i=0; i<corners->size(); i++)
	{
	    float3 temp;
	    temp.z = imgDepthPrevious.at<float>(corners->at(i).x, corners->at(i).y);
	    temp.x = temp.z * (corners->at(i).x - rgb_params_.cx)/rgb_params_.fx;
	    temp.y = -temp.z * (corners->at(i).y - rgb_params_.cy)/rgb_params_.fy;


	    float3 nextTemp;
	    nextTemp.z = imgDepth.at<float>(nextCorners->at(i).x, nextCorners->at(i).y);
	    nextTemp.x = nextTemp.z * (nextCorners->at(i).x - rgb_params_.cx)/rgb_params_.fx;
	    nextTemp.y = -nextTemp.z * (nextCorners->at(i).y - rgb_params_.cy)/rgb_params_.fy;

	    if( !isnan(temp.z) && !isnan(nextTemp.z) )
	    {
		corners3d->push_back(temp);
		nextCorners3d->push_back(nextTemp);
	    }
	}

	for(int i=0; i<corners3d->size(); i++)
	{
	    float3 temp;
	    temp.x = nextCorners3d->at(i).x - corners3d->at(i).x;
	    temp.y = nextCorners3d->at(i).y - corners3d->at(i).y;
	    temp.z = nextCorners3d->at(i).z - corners3d->at(i).z;
	    flows->push_back(temp);
	    // printf("flows, i: %i3, x: %f8, y: %f8, z: %f8 \n", i, temp.x, temp.y, temp.z);
	}
	
	MatrixXd cornersMat(3, corners3d->size());
	MatrixXd nextCornersMat(3, corners3d->size());

    	for(int i=0; i<corners3d->size(); i++)
	{
	    cornersMat(0,i) = corners3d->at(i).x;
	    cornersMat(1,i) = corners3d->at(i).y;
	    cornersMat(2,i) = corners3d->at(i).z;

	    nextCornersMat(0,i) = nextCorners3d->at(i).x;
	    nextCornersMat(1,i) = nextCorners3d->at(i).y;
	    nextCornersMat(2,i) = nextCorners3d->at(i).z;
	}
	
	cout << umeyama(cornersMat, cornersMat, true) << endl;

	

	//Display that image back onto the window
	cv::imshow("Image", imgColor);
	// cv::imshow("Image", imgGrayPrevious);
	
	//Wait...
	cv::waitKey(1);
	if(subImageCount <= 100) {
	    imgGrayPrevious = imgGray.clone();
	    imgDepthPrevious = imgDepth.clone();
	    return;
	}


	

	// imgGrayPrevious = imgGray.clone();
	// imgDepthPrevious = imgDepth.clone();
	delete corners;
	delete nextCorners;
	delete corners3d;
	delete nextCorners3d;



    }

    void convert_rgb_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
	    cv_bridge::CvImageConstPtr cv_image_ptr;
	    cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
	    //cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
	    _img = cv_image_ptr->image;

	}

    void convert_depth_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
	    cv_bridge::CvImageConstPtr cv_image_ptr;
	    //cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
	    cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
	    _img = cv_image_ptr->image;
	}

        

    void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
	    cv_bridge::CvImageConstPtr cv_image_ptr;
	    cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
	    _img = cv_image_ptr->image;
	}



    };

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flash_drive_finder");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Initializing kinectNode...");
    kinectNode kinectNode(nh);
    cout << "Initialization Complete" << endl;

    ros::spin();
    return 0;
}
