# Requirements

 * ROS2:

     - [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) installed on both the robot and the computer.

 * Simulator (only mandatory for the simulation parts, optional for the real robot):

     - [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

 * Libraries and packages:

     - [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) library (for c++) installed on both the robot and the computer.

     - [cv_bridge](https://github.com/ros-perception/vision_opencv/blob/foxy/cv_bridge/README.md#installation) library (for c++) installed on both the robot and the computer.

     - [Zbar](https://zbar.sourceforge.net/download.html) library (for c++) installed on the computer (this is optional, you can use opencv versions instead but you may have to delete some code if you don't have it installed in order to compile the package).

# Getting started

All the nodes and programs of this package were tested using Ubuntu 20.04 x84_64 GNU/Linux and ROS2 Foxy Fitzroy distro.

More information about the process to run the nodes in the [wiki](https://github.com/USanz/follow_beacon/wiki).

