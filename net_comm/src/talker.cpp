/*      talker.cpp
 *      Sends out keyboard events to a ROS topic
 *
 *      Copyright (C) 2011 Debjit Ghosh
 *      Aerial Robotics Team, USC Robotics Society - http://www.uscrs.org - http://uscrs.googlecode.com
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *      
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
**/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <termios.h>
#include <signal.h>

#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44

class OperationKey
{
public:
  OperationKey();
  void inputKey_Loop();

private:

  
  ros::NodeHandle n;
  char keyPressed;
  ros::Publisher chatter_pub;
  
};

OperationKey::OperationKey()
{
  chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
}

int kfd = 0;
struct termios cooked, raw;
  
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void OperationKey::inputKey_Loop()
{
    char c;
    bool dirty=false;

    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the quadrotor.");


    for(;;)
    {
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
            ROS_DEBUG("MOTOR ON");
            keyPressed = 'N';
            dirty = true;
            break;
          case KEYCODE_R:
            ROS_DEBUG("MOTOR OFF");
            keyPressed = 'F';
            dirty = true;
            break;
          case KEYCODE_U:
            ROS_DEBUG("UP");
            keyPressed = 'U';
            dirty = true;
            break;
          case KEYCODE_D:
            ROS_DEBUG("DOWN");
            keyPressed = 'D';
            dirty = true;
            break;
        }

        
          
        if(dirty == true)
        {
            std_msgs::String msg;

            std::stringstream ss;
              
            ss << keyPressed;
            msg.data = ss.str();
              
            ROS_INFO("Key pressed: %s", msg.data.c_str());
            chatter_pub.publish(msg);    
            dirty=false;
        }
    }


  return;
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  
  OperationKey op_key;
  
  signal(SIGINT,quit);
  
  op_key.inputKey_Loop();
  
  return (0);
}
