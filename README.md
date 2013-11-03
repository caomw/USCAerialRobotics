USCAerialRobotics
=================

All code that is used by the quadricopter


The Code is written to interface with the Robot Operating System (ROS). Here is a list of what the different packages do:

art_arduino
Communicates between the onboard computer and the arduino using a serial connection.

kinect
Contains several algorithms taking advantage of kinect data, including an optical flow algorithm and a reference to visual odometry libraries.

icp
Contians algorithms that build on top of the Point Cloud Libraries ICP algorithm, which we use for laser range finder data

art_lrf
Contains a Hough Transform line-finding algorithm for the laser range finder, as well as an odometry algorithm that runs on top of it.
