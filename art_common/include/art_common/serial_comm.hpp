//      SerialComm.hpp
//      Serial communication functionality.
//
//      Copyright (C) 2011 Sam (Yujia Zhai) <yujia.zhai@usc.edu>
//      Aerial Robotics Team, USC Robotics Society - http://www.uscrs.org - http://uscrs.googlecode.com
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

#ifndef _ART_SERIALCOMM_
#define _ART_SERIALCOMM_

#include <ros/ros.h>
#include <exception>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

namespace art
{		
	class SerialComm
	{
	  private:
		int baud;
		int fd;
		bool complete_setting;
		
	  public:
		SerialComm(const char *port, int baud);
		
		~SerialComm();
		
		bool read(char* b);
		
		bool read(char *b, uint32_t max_read_len);
	
		bool write(char* b);
	
		bool write(char* b, uint32_t max_read_len);
	};
}
#endif
