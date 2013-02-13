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

#include <fovis/fovis.hpp>
#include "data_capture.hpp"


#include <vector>

namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace cv;

class kinectNode {

	public:

	private:
		ros::NodeHandle nh;
		ros::Subscriber subImage;
		uint subImageCount;
		
		Mat imgColor;
		Mat imgGray;
		Mat imgGrayPrevious;
		Mat imgCorner;

	public:
	
	kinectNode(ros::NodeHandle& _nh) {
		this->nh = _nh;
		
		subImageCount = 0;
		imgColor = Mat(cv::Size(640, 480), CV_8UC3);
		imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
		imgGray = Mat(cv::Size(640, 480), CV_8UC1);
		
		//subscribe to the image
		//subImage = nh.subscribe("/camera/rgb/image_rect", 10, &kinectNode::subImageCallback, this);
		subImage = nh.subscribe("/camera/depth/image_rect", 10, &kinectNode::subImageCallback, this);

		//Create a cv window to view image in for now
		cv::namedWindow("Image");
	}
	
	~kinectNode() {
		cv::destroyWindow("Image");
		imgColor.release();
		imgGray.release();
	}
	
	private:
	
	void subImageCallback(const sensor_msgs::ImageConstPtr& msg) {
		
		subImageCount++;
		cout << "Got image number: " << subImageCount << endl;
		
		//convert image from ROS image message to OpenCV Mat
		convert_image(msg, imgColor);
		
		//cvtColor(imgColor, imgGray, CV_RGB2GRAY);
		
		cv::imshow("Image", imgColor);
		cv::waitKey(1);
	}

	void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		//cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
		_img = cv_image_ptr->image;
		cv::imshow("raw image", cv_image_ptr->image);
		cv::waitKey(10);
	}
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "flash_drive_finder");
	ros::NodeHandle nh;
	
	cout << "Initializing kinectNode..." << endl;
	kinectNode kinectNode(nh);
	cout << "Initialization Complete" << endl;
	
	ros::spin();
	return 0;
}


void updateGradients(float* ptrX, float* ptrY, pitch, yaw2)
{
    float totalGrad = tan(pitch);
    *ptrX = cos(yaw2)*totalGrad;
    *ptrY = sin(yaw2)*totalGrad;
    return;
}

void RANSAC(vector<pointCorrespondence> flowVectors, float SQR_ERROR, vector<float> &estTrans)
{
    int maxInliers = 0;
    vector<int> bestMatches;
    
    for(int i=0; i< pointCorrespondence.size(); i++)
    {
	int currentInliers = 0;
	vector<int> matches;
	for(int j=0; j<pointCorrespondence.size(); j++)
	{
	    if(pow(pow(flowVectors[i].distX - flowVectors[j].distX, 2) + pow(flowVectors[i].distY - flowVectors[j].distY, 2) + pow(flowVectors[i].distZ - flowVectors[j].distZ, 2),0.5) < SQR_ERROR)
	    {
		currentInliers++;
		matches.push_back(j);
	    }
	}
	if(currentInliers > maxInliers)
	{
	    maxInliers = currentInliers;
	    bestMatches = matches;
	}
    }
    

    for(int i=0; i<bestMatches.size(); i++)
    {
	estTrans[0] += flowVectors[i].distX;
	estTrans[1] += flowVectors[i].distY;
	estTrans[2] += flowVectors[i].distZ;
    }
    estTrans[0] = estTrans[0]/bestMatches.size();
    estTrans[1] = estTrans[1]/bestMatches.size();
    estTrans[2] = estTrans[2]/bestMatches.size();
}



    
void planeProjection(float yaw2, float pitch, float optDistPlane, int u, int v, float alpha_x, float alpha_y, int pixelX, int pixelY, float* egoX, float* egoY, float* egoZ)
{

    //input yaw, pitch, yaw2, and optDistPlane for each of the two frames and delta_yaw between frames
    float yaw2 = 0;
    float pitch = 0;
    float optDistPlane = 1;

    int u = 240;
    int v = 320;
    float alpha_x = 1;
    float alpha_y = 1;

    float gradX;
    float gradY;

    int pixelX;
    int pixelY;

    float imagePlaneX;
    float imagePlaneY;
    
    imagePlaneX = (pixelX-u)*alpha_x;
    imagePlaneY = (pixelY-v)*alpha_y;

    updateGradients(&gradX, &gradY, pitch, yaw2);

    if(1 - imagePlaneX*gradX - imagePlaneY*gradY <= 0)
    {
	return 0;
    }
    *egoZ = optDistPlane/(1 - imagePlaneX*gradX - imagePlaneY*gradY);
    *egoY = imagePlaneY* (*egoZ);
    *egoX = imagePlaneX* (*egoZ);
}

