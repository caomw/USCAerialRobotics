#include <art_common/serial_comm.hpp>

using namespace std;

namespace art
{
	SerialComm::SerialComm(const char *port, int baud): baud(baud), fd(0), complete_setting(false)
	{
		/** try opening the port **/
		fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
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
		while (this->read((char*) &b) > 0) { }
		complete_setting = true;
	}
	
	SerialComm::~SerialComm()
	{
		if(fd > 0)
		close(fd);
		fd = 0;
	}
	
	bool SerialComm::read(char* b)
	{
		if(!complete_setting || fd < 0 || ::read(fd, (uint8_t*) b, 1) < 0)
			return false;
		return true;
	}
	
	bool SerialComm::read(char *b, uint32_t max_read_len)
	{
		if(!complete_setting || fd < 0 || ::read(fd, (uint8_t*) b, (size_t) max_read_len) < 0)
		return true;
	}

	bool SerialComm::write(char* b)
	{
		if(!complete_setting || fd < 0 || ::write(fd, (uint8_t*) b, 1) < 0)
			return false;
		return true;
	}

	bool SerialComm::write(char* b, uint32_t max_read_len)
	{
		tcflush(fd, TCOFLUSH);
		if(!complete_setting || fd < 0 || ::write(fd, (uint8_t*) b, (size_t) max_read_len) < 0)
			return false;
		return true;
	}
}
