#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <termios.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
   
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);
  
  //ros::Rate loop_rate(10);
  
  int count = 0;
  int kfd = 0;
  
  struct termios cooked, raw;
  
  
  while (ros::ok())
  {  
    char c;
    char keyPressed;
    // get the console in raw mode       
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Keyboard listener");
    puts("---------------------------");
    puts("Use arrow keys to move the quadrotor.");
    
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

    ROS_DEBUG("value: 0x%02X\n", c);
    
    switch(c)
      {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        keyPressed = 'L';
        break;
          
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        keyPressed = 'R';
        break;
        
      case KEYCODE_U:
        ROS_DEBUG("UP");
        keyPressed = 'U';
        break;
          
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        keyPressed = 'D';
        break;
      }
     
    std_msgs::String msg;

    std::stringstream ss;
      
    ss << " keyboard character:" << keyPressed;
    msg.data = ss.str();
      
    ROS_INFO("%s", msg.data.c_str());
      
    chatter_pub.publish(msg);
      
    //ros::spinOnce();
      
    //loop_rate.sleep();
      
    ++count;
  }


  return 0;
}
