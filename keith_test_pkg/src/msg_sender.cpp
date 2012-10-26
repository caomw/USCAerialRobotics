#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

ros::Publisher pub_output;
ros::Subscriber sub_input;

int counter = 0;

void cb_input (const std_msgs::String::ConstPtr msg) {
  cout << "Got message: " << msg->data << endl;
  counter ++;
  
  stringstream ss;
  ss << counter;

  std_msgs::String msg_pub;
  msg_pub.data = ss.str();

  pub_output.publish(msg_pub);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "msg_sender_node");
  ros::NodeHandle nh;
  pub_output = nh.advertise<std_msgs::String>("/output", 10);
  sub_input = nh.subscribe("/input", 10, cb_input);
  while(ros::ok()) {
    ros::spinOnce();
  }
}

