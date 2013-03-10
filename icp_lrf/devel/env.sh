#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/russell/ros/USCAerialRobotics/icp_lrf/devel', type 'exit' to leave"
  . "/home/russell/ros/USCAerialRobotics/icp_lrf/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/russell/ros/USCAerialRobotics/icp_lrf/devel'"
else
  . "/home/russell/ros/USCAerialRobotics/icp_lrf/devel/setup.sh"
  exec "$@"
fi
