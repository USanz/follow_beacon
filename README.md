# Introduction

This repository contains different ways to make a turtlebot3 follow a beacon, which in this cas is a QR code:
 - ROS2 package, in which we can see the following evolution:
   [color filter](https://github.com/USanz/follow_beacon/wiki/2.-Real-robot-color-follower) -> [QR code detector](https://github.com/USanz/follow_beacon/wiki/3.-Real-robot-QR-code-follower) -> [QR code detector using zenoh bridge](https://github.com/USanz/follow_beacon/wiki/3.-Real-robot-QR-code-follower#running-it-using-zenoh-zenoh-bridge-dds).
 - [Zenoh nodes](https://github.com/USanz/follow_beacon/wiki/4.-Real-robot-QR-code-follower-using-Zenoh).
 - [Zenoh-flow nodes](https://github.com/USanz/follow_beacon/wiki/5.-Real-robot-QR-code-follower-using-Zenoh-flow).

There's more information about this project and how to run it in the [getting started section](#getting-started)

# Requirements

 * ROS2:

     - [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) installed on both the robot and the computer.

 * Simulator (only mandatory for the simulation parts, optional for the real robot):

     - [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

 * Libraries and packages:

     - [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) library (for c++) installed on both the robot and the computer.

     - [cv_bridge](https://github.com/ros-perception/vision_opencv/blob/foxy/cv_bridge/README.md#installation) library (for c++) installed on both the robot and the computer.

     - [Zbar](https://zbar.sourceforge.net/download.html) library (for c++) installed on the computer (this is optional, you can use opencv versions instead but you may have to delete some code if you don't have it installed in order to compile the package).

 * For the zenoh nodes (not necessary for the ROS2 nodes as they use DDS):

     - [Zenoh-bridge-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds#readme)

     - [Zenoh router](https://zenoh.io/docs/getting-started/installation/)

     - [Zenoh-python API](https://github.com/eclipse-zenoh/zenoh-python#readme)

     - [Zenoh-flow](https://github.com/eclipse-zenoh/zenoh-flow#readme)

     - [Zenoh-flow-python](https://github.com/eclipse-zenoh/zenoh-flow-python#readme)

# Getting started

All the nodes and programs of this package were tested using Ubuntu 20.04 x84_64 GNU/Linux and ROS2 Foxy Fitzroy distro.

More information about the process to run the nodes in the [wiki](https://github.com/USanz/follow_beacon/wiki).

Feel free to use this QR code to test:

![QR code](./qr_code.png)