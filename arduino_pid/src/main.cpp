#include <ros/ros.h>
#include <art_common/serial_comm.hpp>

using namespace std;

class ArduinoPID
{
	ros::NodeHandle& nh;
	
	union Packet
	{
		struct
		{
			float roll[3];
			float pitch[3];
			float yaw[3];
			float altitude[3];
		};
		char data[48];
	};
	
	double roll[3], pitch[3], yaw[3], altitude[3]; /// in the order of p i d
	double roll_d[3], pitch_d[3], yaw_d[3], altitude_d[3];
	art::SerialComm serial;
	ros::Rate rt;
	
	void getparams()
	{
		nh.param("proll", roll[0], 1.5);
		nh.param("iroll", roll[1], 3.0);
		nh.param("droll", roll[2], 1.5);
		nh.param("ppitch", pitch[0], 1.5);
		nh.param("ipitch", pitch[1], 3.0);
		nh.param("dpitch", pitch[2], 1.5);
		nh.param("pyaw", yaw[0], 2.0);
		nh.param("iyaw", yaw[1], 0.0);
		nh.param("dyaw", yaw[2], 0.0);
		nh.param("paltitude", altitude[0], 0.9);
		nh.param("ialtitude", altitude[1], 0.8);
		nh.param("daltitude", altitude[2], 0.9);
	}
	
	void setupdate()
	{
		for(int i = 0; i < 3; i++)
		{
			roll_d[i] = roll[i];
			pitch_d[i] = pitch[i];
			yaw_d[i] = yaw[i];
			altitude_d[i] = altitude[i];
		}
		
		Packet p;
		for(int i = 0; i < 3; i++)
		{
			p.roll[i] = static_cast<float>(roll[i]);
			p.pitch[i] = static_cast<float>(pitch[i]);
			p.yaw[i] = static_cast<float>(yaw[i]);
			p.altitude[i] = static_cast<float>(altitude[i]);
		}
		serial.write(p.data, 48);
	}
	
	bool checkupdate()
	{
		for(int i = 0; i < 3; i++)
		{
			if(roll_d[i] != roll[i]) return true;
			if(pitch_d[i] != pitch[i]) return true;
			if(yaw_d[i] != yaw[i]) return true;
			if(altitude_d[i] != altitude[i]) return true;
		}
		return false;
	}
	
	void printupdate()
	{
		cout << endl << " - UPDATE - P I D - ";
		cout << endl << "    Roll: \t";
		for(int i = 0; i < 3; i ++)
			cout << roll[i] << "\t";
		cout << endl << "    Pitch: \t";
		for(int i = 0; i < 3; i ++)
			cout << pitch[i] << "\t";
		cout << endl << "    Yaw: \t";
		for(int i = 0; i < 3; i ++)
			cout << yaw[i] << "\t";
		cout << endl << " Altitude: \t";
		for(int i = 0; i < 3; i ++)
			cout << altitude[i] << "\t";
		cout << endl;
	}
	
  public:
	
	ArduinoPID(ros::NodeHandle& _nh): nh(_nh), rt(1), serial("/dev/ttyUSB0", 115200)
	{
		ros::Duration(2.0).sleep();
		while(ros::ok())
		{
			getparams();
			if(checkupdate())
			{
				rt.sleep();
				setupdate();
				printupdate();
			}
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arduino_pid");
	ros::NodeHandle nh;
	ArduinoPID ap(nh);
	return 0;
}
