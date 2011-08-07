#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <art_common/KinectMsg.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Pose2D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

int standard_length = 85;
#define ToRad(x) x/180 * 3.1415926535

using namespace std;
namespace art_window
{	
	class WindowPose : public nodelet::Nodelet
	{
		ros::NodeHandle nh;
		ros::Subscriber sub_kinect;
		ros::Publisher pub_points, pub_pose;
		
		struct PARAM_hough
		{
			int rho_res, theta_res, hough_thresh, min_length, max_gap;
			PARAM_hough(): rho_res(1), theta_res(0), hough_thresh(20), min_length(20), max_gap(8) {}
		} param_hough;

		struct PARAM_CANNY
		{
			int threshold1, threshold2;
			PARAM_CANNY(): threshold1(100), threshold2(200) {}
		} param_canny;

		struct Line
		{
			int loc_a[2], loc_b[2];
			float point_a[3], point_b[3];
			float length;
			float vector[3];
		};

	  private:
		virtual void onInit()
		{
			/// Init variables.
			
			/// Start messaging.
			nh = getNodeHandle();

			cv::namedWindow("haha");
			cv::createTrackbar("length", "haha", &standard_length, 400);
			
			pub_points = nh.advertise<sensor_msgs::PointCloud2>("/points", 10);
			pub_pose = nh.advertise<geometry_msgs::Pose2D>("/arduino/robot_pose", 10);
			sub_kinect = nh.subscribe("/kinect/raw", 10, &WindowPose::cb_sub_kinect, this);
		}

		void cb_sub_kinect(const art_common::KinectMsg::ConstPtr msg)
		{
			cv::Mat depth;
			cv::Mat edges, edges_dilated;
			cv::Mat buf1, buf2, buf3;
			cv::Mat display1, display2;

			/// Crop and threshold the image. Size: 200 (y,row) x 290 (x,col). Thresh: 5120;
			{
				depth = cv::Mat::zeros(200, 290, CV_8UC1);
				short* msg_ptr = (short*) &(msg->depth[0]) + 320 * 20 + 15;
				uchar* depth_ptr = &(depth.data[0]);
				for(uint i = 0; i < 200; i++)
				{
					for(uint j = 0; j < 290; j++, msg_ptr++, depth_ptr++)
						if(*msg_ptr < 5120) *depth_ptr = static_cast<uchar>(*msg_ptr / 20);
					msg_ptr += 30;
				}
			}

			/// Border extraction and filtering.
			cv::Canny(depth, edges, param_canny.threshold1, param_canny.threshold2);
			cv::blur(edges, buf1, cv::Size(3,3));
			cv::threshold(buf1, edges_dilated, 0, 255, cv::THRESH_BINARY);

			/// Init display.
			display1 = cv::Mat::zeros(200, 290, CV_8UC3);
			display2 = cv::Mat::zeros(200, 290, CV_8UC3);
			
			/// Line detection.
			vector<Line> lines_parallel, lines_vertical;
			{
				std::vector<cv::Vec4i> hough_lines;
				cv::HoughLinesP(edges_dilated, hough_lines, std::max(0.1, double(param_hough.rho_res)), std::max(0.1,param_hough.theta_res * M_PI/180.0), std::max(1.0, double(param_hough.hough_thresh)), std::max(1, param_hough.min_length), std::max(1, param_hough.max_gap));

				for(uint i = 0; i < hough_lines.size(); i++)
				{
					Line nl;
					if(!(get_closest_loc(nl.loc_a, depth, edges, hough_lines[i][0], hough_lines[i][1]) && get_closest_loc(nl.loc_b, depth, edges, hough_lines[i][2], hough_lines[i][3]))) continue;
					get_3d_point(nl.point_a, msg, nl.loc_a[0], nl.loc_a[1]);
					get_3d_point(nl.point_b, msg, nl.loc_b[0], nl.loc_b[1]);
	
					nl.length = 0;
					for(int j = 0; j < 3; j++)
					{
						nl.length += pow(nl.point_a[j] - nl.point_b[j], 2);
						nl.vector[j] = nl.point_b[j] - nl.point_a[j];
					}

					nl.length = sqrt(nl.length);
					{
						if((abs(nl.loc_a[0] - nl.loc_b[0]) > 20) && (abs(nl.loc_a[1] - nl.loc_b[1]) < 15))
							lines_parallel.push_back(nl);
						else if((abs(nl.loc_a[0] - nl.loc_b[0]) < 15) && (abs(nl.loc_a[1] - nl.loc_b[1]) > 20))
							lines_vertical.push_back(nl);
					}
				}
			}

			if(lines_parallel.size() == 0 || lines_vertical.size() == 0) return;

			Line bottom_line, left_line, right_line;
			bool found_bottom_line = false, found_left_line = false, found_right_line = false;

			vector<Line> bottom_lines;
			/// First, get the bottom line.
			{
				for(uint i = 0; i < lines_parallel.size(); i++)
				{
					if(nl.length > (target_length * 0.7) && nl.length < (target_length * 1.3))
						bottom_lines.push_back(lines_parallel[i]);
				}
			}

			if(!found_bottom_line) return;

			/// Then, get the left / right lines.
			{
				float left_distance = 50, right_distance = 50;
				for(uint i = 0; i < lines_vertical.size(); i++)
				{
					float vertical_loc = (lines_vertical[i].loc_a[0] + lines_vertical[i].loc_b[0])/2;
					if(abs(max(lines_vertical[i].loc_a[1], lines_vertical[i].loc_b[1]) - (bottom_line.loc_a[1] + bottom_line.loc_b[1])/2) < 40)
					{
						if(vertical_loc < (bottom_line.loc_a[0] + 20))
						{
							if((bottom_line.loc_a[0] - vertical_loc) < left_distance)
							{
								found_left_line = true;
								left_line = lines_vertical[i];
								left_distance = bottom_line.loc_a[0] - vertical_loc;
							}
						}
						else if(vertical_loc > (bottom_line.loc_b[0] - 20))
						{
							if((vertical_loc - bottom_line.loc_b[0]) < right_distance)
							{
								found_right_line = true;
								right_line = lines_vertical[i];
								right_distance = vertical_loc - bottom_line.loc_b[0];
							}
						}
					}
				}
			}

			
			if(!(found_left_line && found_right_line)) return;			

			cv::Vec4i final_line;
			{
				cv::Vec4i tl(bottom_line.loc_a[0], bottom_line.loc_a[1], bottom_line.loc_b[0], bottom_line.loc_b[1]);
				cv::Vec4i ll(left_line.loc_a[0], left_line.loc_a[1], left_line.loc_b[0], left_line.loc_b[1]);
				cv::Vec4i rl(right_line.loc_a[0], right_line.loc_a[1], right_line.loc_b[0], right_line.loc_b[1]);
				
				float a1 = static_cast<float>(tl[2] - tl[0]) / (tl[3] - tl[1]);
				float b1 = static_cast<float>(tl[0]) - a1 * tl[1];
				{
					float a2 = static_cast<float>(ll[2] - ll[0]) / (ll[3] - ll[1]);
					float b2 = static_cast<float>(ll[0]) - a2 * ll[1];
					float y = (b1 - b2) / (a2 - a1);
					final_line[1] = static_cast<int>(y);
					final_line[0] = static_cast<int>(a1 * y + b1);
				}
				{
					float a2 = static_cast<float>(rl[2] - rl[0]) / (rl[3] - rl[1]);
					float b2 = static_cast<float>(rl[0]) - a2 * rl[1];
					float y = (b1 - b2) / (a2 - a1);
					final_line[3] = static_cast<int>(y);
					final_line[2] = static_cast<int>(a1 * y + b1);
				}
			}

			for(int i = 0; i < 4; i++)
			{
				if(final_line[i] <= 0) return;
			}
			if(final_line[0] >= 290) return;
			if(final_line[1] >= 200) return;
			if(final_line[2] >= 290) return;
			if(final_line[3] >= 200) return;
			

			Line real_line;

			if(get_closest_loc(real_line.loc_a, depth, edges,  final_line[0], final_line[1], 20)
				&& get_closest_loc(real_line.loc_b, depth, edges, final_line[2], final_line[3], 20))
			{
				ROS_INFO("haha");
				get_3d_point(real_line.point_a, msg, real_line.loc_a[0], real_line.loc_a[1]);
				get_3d_point(real_line.point_b, msg, real_line.loc_b[0], real_line.loc_b[1]);

				for(int i = 0; i < 3; i++)
				{
					cout << real_line.point_a[i] << "\t" << real_line.point_b[i] << endl;
				}

				float translation[3];
				for(int i = 0; i < 3; i++)
				{
					translation[i] = (real_line.point_a[i] + real_line.point_b[i]) * 0.5;
				}
				ROS_INFO("before sending to tf");
				static tf::TransformBroadcaster br;
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(translation[0], translation[1], translation[2]));
				transform.setRotation(tf::Quaternion(0, 0, 0));
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/window"));

				geometry_msgs::Pose2D pose;
				pose.x = static_cast<double>(translation[0]);
				pose.y = static_cast<double>(-translation[2]);
				pub_pose.publish(pose);
			}

			pcl::PointCloud<pcl::PointXYZ> points;
			for(int i = 0; i < 200; i++)
			{
				for(int j = 0; j < 290; j++)
				{
					if(edges.at<uchar>(i,j) == 255)
					{
						float pos[3];
						get_3d_point(pos, msg, j, i);
						points.points.push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
					}
				}
			}
			sensor_msgs::PointCloud2 points_msg;
			pcl::toROSMsg(points, points_msg);
			points_msg.header.frame_id = "/world";
			points_msg.header.stamp = ros::Time::now();
			pub_points.publish(points_msg);
			
			/// Plot.

			//cv::line(display2, cv::Point(bottom_line.loc_a[0], bottom_line.loc_a[1]), cv::Point(bottom_line.loc_b[0], bottom_line.loc_b[1]), cv::Scalar(255,0,0), 3, 8);
			
			cv::line(display1, cv::Point(final_line[0], final_line[1]), cv::Point(final_line[2], final_line[3]), cv::Scalar(255,0,0), 3, 8);

			cv::line(display1, cv::Point(left_line.loc_a[0], left_line.loc_a[1]), cv::Point(left_line.loc_b[0], left_line.loc_b[1]), cv::Scalar(0,255,0), 3, 8);

			cv::line(display1, cv::Point(right_line.loc_a[0], right_line.loc_a[1]), cv::Point(right_line.loc_b[0], right_line.loc_b[1]), cv::Scalar(0,255,0), 3, 8);
			

			for(int i = 0; i < lines_vertical.size(); i++)
			{
				cv::line(display2, cv::Point(lines_vertical[i].loc_a[0], lines_vertical[i].loc_a[1]), cv::Point(lines_vertical[i].loc_b[0], lines_vertical[i].loc_b[1]), cv::Scalar(0,255,0), 3, 8);
			}

			/*for(int i = 0; i < lines_parallel.size(); i++)
			{
				cv::line(display2, cv::Point(lines_parallel[i].loc_a[0], lines_parallel[i].loc_a[1]), cv::Point(lines_parallel[i].loc_b[0], lines_parallel[i].loc_b[1]), cv::Scalar(255,0,0), 3, 8);
			}*/

			for(int i = 0; i < bottom_lines.size(); i++)
			{
				cv::line(display2, cv::Point(bottom_lines[i].loc_a[0], bottom_lines[i].loc_a[1]), cv::Point(bottom_lines[i].loc_b[0], bottom_lines[i].loc_b[1]), cv::Scalar(255,0,0), 3, 8);
			}

			cv::imshow("haha", display1);
			cv::imshow("ref", display2);
			cv::waitKey(50);
		}

		void get_3d_point(float* point, const art_common::KinectMsg::ConstPtr msg, int x, int y)
		{
			float Z = static_cast<float>(((const short*)(&msg->depth[0]))[(y+20) * 320 + (x+15)]) * 0.001;
			point[0] = (x - 144.5) * Z * 0.003809524;
			point[1] = (y - 99.5) * Z * 0.003809524;
			point[2] = Z;
		}

		bool get_closest_loc(int* point, cv::Mat depth, cv::Mat edges, int x, int y, int blocksize = 5)
		{
			bool found = false; point[0] = 0; point[1] = 0;	int square = 0; uint min_depth = 0;
			for(int i = blocksize; i >= - blocksize; i--)
			{
				for(int j = blocksize; j >= - blocksize; j--)
				{
					if((edges.at<uchar>(y+j,x+i) == 255) && (depth.at<uchar>(y+j,x+i) != 0))
					{
						if((!found))
						{
							found = true;
							point[0] = i; point[1] = j; square = j*j+i*i; min_depth = depth.at<uchar>(y+j,x+i);
						}
						else if((depth.at<uchar>(y+j,x+i) < min_depth * 1.1) && (j*j+i*i < square))
						{
							point[0] = i; point[1] = j; square = j*j+i*i; min_depth = depth.at<uchar>(y+j,x+i);
						}
					}		
				}
			}
			point[0] += x;
			point[1] += y;
			return found;
		}
	};
}

PLUGINLIB_DECLARE_CLASS(art_window, window_pose, art_window::WindowPose, nodelet::Nodelet);
