#include "ros/ros.h"
#include "net_comm/Altitude.h"
#include "net_comm/SetHoverAltitude.h"
#include "net_comm/LiftOff.h"
#include "net_comm/Land.h"
#include "net_comm/ArmMotors.h"
#include "net_comm/DisarmMotors.h"


bool set_hover_altitude(net_comm::SetHoverAltitude::Request  &req,
                        net_comm::SetHoverAltitude::Response &res )
{
  ROS_INFO("Set hover to z=%d.", req.alt);	
  // Send serial command to hover
  ros::Duration(1).sleep();
  // 
  return true;
}

bool lift(net_comm::LiftOff::Request  &req,
          net_comm::LiftOff::Response &res)
{
  ROS_INFO("Liftoff!");	
  // Send serial command to liftoff
  ros::Duration(1).sleep();
  //
  return true;
}

bool land(net_comm::Land::Request  &req,
          net_comm::Land::Response &res)
{
  ROS_INFO("Landing.");	
  // Send serial command to land
  ros::Duration(1).sleep();
  //
  return true;
}

bool arm_motors(net_comm::ArmMotors::Request  &req,
                net_comm::ArmMotors::Response &res)
{
  ROS_INFO("Arming Motors.");	
  // Send serial command to arm motors
  ros::Duration(1).sleep();
  //
  return true;
}

bool disarm_motors(net_comm::DisarmMotors::Request  &req,
                   net_comm::DisarmMotors::Response &res)
{
  ROS_INFO("Disarming Motors.");	
  // Send serial command to disarm motors
  ros::Duration(1).sleep();
  //
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_server");
  ros::NodeHandle n;

  ros::ServiceServer s_set_hover_altitude = n.advertiseService("serializer/set_hover_altitude", set_hover_altitude);
  ros::ServiceServer s_lift = n.advertiseService("serializer/lift_off", lift);
  ros::ServiceServer s_land = n.advertiseService("serializer/land", land);
  ros::ServiceServer s_arm_motors = n.advertiseService("serializer/arm_motors", arm_motors);
  ros::ServiceServer s_disarm_motors = n.advertiseService("serializer/disarm_motors", disarm_motors);
  ROS_INFO("Serializer ready.");
  ros::spin();

  return 0;
}