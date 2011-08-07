#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <art_arduino/PIDConfig.h>

using namespace std;

class PIDTuning
{
	ros::NodeHandle& nh;
	dynamic_reconfigure::Server<art_arduino::PIDConfig> srv;
	
	
	void reconfigure_callback(art_arduino::PIDConfig &config, uint32_t level)
	{
		if(!ros::ok()) return;

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
	
  public:
	PIDTuning(ros::NodeHandle& _nh): nh(_nh)
	{
		ros::Duration(3.0).sleep();
		dynamic_reconfigure::Server<art_arduino::PIDConfig>::CallbackType f;
		f = boost::bind(&PIDTuning::reconfigure_callback, this, _1, _2);
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
