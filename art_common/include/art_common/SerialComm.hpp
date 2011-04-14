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

#include <exception>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

class SerialComm
{
  private:
	int baud;
	int fd;
	bool complete_setting;
	
  public:
	SerialComm(const char *port, int baud): baud(baud), fd(0), complete_setting(false)
	{
		/** try opening the port **/
		fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (fd < 0){ throw; return; }

		/** setup new configuration */
		struct termios oldtio, newtio;
		if(tcgetattr(fd, &oldtio) < 0){ throw; return; }
		bzero(&newtio, sizeof(newtio)); // bzero: flush 0 into the variable
		newtio.c_iflag = IGNPAR | INPCK;
		newtio.c_oflag = 0;
		newtio.c_cflag = CS8 | CLOCAL | CREAD;
		newtio.c_lflag = 0;
		newtio.c_cc[VTIME] = 0;
		newtio.c_cc[VMIN] = 0;
		cfsetspeed(&newtio, baud);
		tcflush(fd, TCIOFLUSH);
		if (tcsetattr(fd, TCSANOW, &newtio) < 0){ throw; return; }
		
		/** flush (clear) the buffer of the serial device **/
		uint8_t b;
		while (this->read(&b) > 0) { }
		complete_setting = true;
	}
	
	~SerialComm()
	{
		if(fd > 0)
		close(fd);
		fd = 0;
	}
	
	bool read(char* b)
	{
		if(!complete_setting || fd < 0 || ::read(fd, (uint8_t*) b, 1) < 0)
			return false;
		return (nread == 1);
	}
	
	bool read(char *b, uint32_t max_read_len)
	{
		if(!complete_setting || fd < 0 || ::read(fd, (uint8_t*) b, (size_t) max_read_len) < 0)
		return true;
	}

	bool write(char* b)
	{
		if(!complete_setting || fd < 0 || write(fd, (uint8_t*) b, 1) < 0)
			return false;
		return true;
	}

	bool write(char* b, uint32_t max_read_len)
	{
		if(!complete_setting || fd < 0 || ::write(fd, (uint8_t*) block, (size_t) max_read_len) < 0)
			return false;
		tcflush(fd, TCOFLUSH);
		return true;
	}
};

#endif
