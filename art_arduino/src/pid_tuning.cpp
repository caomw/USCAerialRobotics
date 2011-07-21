#include <ros/ros.h>
#include <art_common/serial_comm.hpp>
#include <dynamic_reconfigure/server.h>
#include <art_arduino/PIDConfig.h>

using namespace std;

class PIDTuning
{
	ros::NodeHandle& nh;
	dynamic_reconfigure::Server<art_arduino::PIDConfig> srv;
	
	union Packet
	{
		float pid_params[12];
		char data[48];
	};
	
	double pid_params[12];
	art::SerialComm serial;
	
	void pid_callback(art_arduino::PIDConfig &config, uint32_t level)
	{
		if(! ros::ok()) return;

		bool updated = false;
		if(config.proll != pid_params[0]) { updated = true; pid_params[0] = config.proll; }
		if(config.iroll != pid_params[1]) { updated = true; pid_params[1] = config.iroll; }
		if(config.droll != pid_params[2]) { updated = true; pid_params[2] = config.droll; }
		if(config.ppitch != pid_params[3]) { updated = true; pid_params[3] = config.ppitch; }
		if(config.ipitch != pid_params[4]) { updated = true; pid_params[4] = config.ipitch; }
		if(config.dpitch != pid_params[5]) { updated = true; pid_params[5] = config.dpitch; }
		if(config.pyaw != pid_params[6]) { updated = true; pid_params[6] = config.pyaw; }
		if(config.iyaw != pid_params[7]) { updated = true; pid_params[7] = config.iyaw; }
		if(config.dyaw != pid_params[8]) { updated = true; pid_params[8] = config.dyaw; }
		if(config.paltitude != pid_params[9]) { updated = true; pid_params[9] = config.paltitude; }
		if(config.ialtitude != pid_params[10]) { updated = true; pid_params[10] = config.ialtitude; }
		if(config.daltitude != pid_params[11]) { updated = true; pid_params[11] = config.daltitude; }
		
		if(updated)
		{
			setupdate();
			printupdate();
			system("rosparam dump $(rospack find art_arduino)/pid.yaml /arduino_pid_tuning_node");
		}
	}
	
	void setupdate()
	{
		Packet p;
		for(int i = 0; i < 12; i++) p.pid_params[i] = static_cast<float>(pid_params[i]);
		serial.write(p.data, 48);
	}
	
	void printupdate()
	{
		cout << endl << " - UPDATE - P I D - ";
		cout << endl << "    Roll: \t";
		for(int i = 0; i < 3; i ++)
			cout << pid_params[i] << "\t";
		cout << endl << "    Pitch: \t";
		for(int i = 3; i < 6; i ++)
			cout << pid_params[i] << "\t";
		cout << endl << "    Yaw: \t";
		for(int i = 6; i < 9; i ++)
			cout << pid_params[i] << "\t";
		cout << endl << " Altitude: \t";
		for(int i = 9; i < 12; i ++)
			cout << pid_params[i] << "\t";
		cout << endl;
	}
	
  public:
	
	PIDTuning(ros::NodeHandle& _nh): nh(_nh), serial("/dev/ttyUSB0", 115200)
	{
		ros::Duration(3.0).sleep();
		dynamic_reconfigure::Server<art_arduino::PIDConfig>::CallbackType f;
		f = boost::bind(&PIDTuning::pid_callback, this, _1, _2);
		srv.setCallback(f);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arduino_pid_tuning_node");
	ros::NodeHandle nh;
	PIDTuning pid_tuning(nh);
	ros::spin();
	return 0;
}
