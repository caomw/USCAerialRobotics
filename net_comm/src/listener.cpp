/*      listener.cpp
 *      Listens to keyboard events and writes to serial port
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
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
using namespace std;

int fd1;
int fd2;
char *buff,*buffer,*bufptr;
int wr,rd,nbytes,tries;

void chatterCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard you man: [%s]", msg->data.c_str());
  
  if(msg->data == "N")
  {
    ROS_INFO("MOTOR ON");
    //Open device
    fd1 = open("dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); 
    
    if(fd1 == -1)
    {
        printf("Error opening port");
        perror("open_port: Unable to open /dev/ttyS0 - ");
    }	
    else
    {
        fcntl(fd1, F_SETFL,0);
        printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
    }
    
    
    wr=write(fd1,"21000\r",4);
  }
    
  else if(msg->data == "F")
  {
    ROS_INFO("MOTOR OFF[%s]");
    //Open device
    fd1 = open("dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); 
    
    if(fd1 == -1)
    {
        printf("Error opening port");
        perror("open_port: Unable to open /dev/ttyS0 - ");
    }	
    else
    {
        fcntl(fd1, F_SETFL,0);
        printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
    }
    
    
    wr=write(fd1,"20000",4);
  }
  
  else if(msg->data == "U")
  {
    ROS_INFO("MOTOR OFF[%s]");
    
    fd1 = open("dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); 
    
    if(fd1 == -1)
    {
        printf("Error opening port");
        perror("open_port: Unable to open /dev/ttyS0 - ");
    }	
    else
    {
        fcntl(fd1, F_SETFL,0);
        printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
    }
    
    
    wr=write(fd1,"3BC\r",4);
  }
  
  else if(msg->data == "D")
  {
    ROS_INFO("MOTOR OFF[%s]");
    
    fd1 = open("dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY); 
    
    if(fd1 == -1)
    {
        printf("Error opening port");
        perror("open_port: Unable to open /dev/ttyS0 - ");
    }	
    else
    {
        fcntl(fd1, F_SETFL,0);
        printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
    }
    
    
    wr=write(fd1,"30000",4);
  }
  
  
}
