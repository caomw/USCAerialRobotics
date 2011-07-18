/*
 * Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include "std_msgs/String.h"

#include <boost/thread/thread.hpp>
#include <ros/ros.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

class ErraticKeyboardTeleopNode
{
    private:
        
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        ErraticKeyboardTeleopNode()
        {
            pub_ = n_.advertise<std_msgs::String>("chatter", 1);
        }
        
        ~ErraticKeyboardTeleopNode() { }
        void keyboardLoop();
        
        
};

ErraticKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"talker");//, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    ErraticKeyboardTeleopNode tbk;
    
    boost::thread t = boost::thread(boost::bind(&ErraticKeyboardTeleopNode::keyboardLoop, &tbk));
    
    ros::spin();
    
    t.interrupt();
    t.join();
    //tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);
    
    return(0);
}

void ErraticKeyboardTeleopNode::keyboardLoop()
{
    char c;
    char keyPressed;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("Use arrow keys to control the robot");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    
    while(ros::ok)
    {
        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            puts("Entered");
            exit(-1);
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                puts("entered");
                exit(-1);
            }
        }
        
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
          
        pub_.publish(msg);
        
    }
}
