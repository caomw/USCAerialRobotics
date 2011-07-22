#include "ros/ros.h"
#include "net_comm/Altitude.h"
#include "net_comm/SetHoverAltitude.h"
#include "net_comm/LiftOff.h"
#include "net_comm/Land.h"


bool set_hover_altitude(net_comm::SetHoverAltitude::Request  &req,
                        net_comm::SetHoverAltitude::Response &res )
{
  ROS_INFO("Set hover to z=%d.", req.alt);	
  // Send serial command to hover
  return true;
}

bool lift(net_comm::LiftOff::Request  &req,
          net_comm::LiftOff::Response &res)
{
  ROS_INFO("Liftoff!");	
  // Send serial command to liftoff
  return true;
}

bool land(net_comm::Land::Request  &req,
          net_comm::Land::Response &res)
{
  ROS_INFO("Landing.");	
  // Send serial command to land
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_server");
  ros::NodeHandle n;

  ros::ServiceServer s_set_hover_altitude = n.advertiseService("serializer/set_hover_altitude", set_hover_altitude);
  ros::ServiceServer s_lift = n.advertiseService("serializer/lift_off", lift);
  ros::ServiceServer s_land = n.advertiseService("serializer/land", land);
  ROS_INFO("Serializer ready.");
  ros::spin();

  return 0;
}