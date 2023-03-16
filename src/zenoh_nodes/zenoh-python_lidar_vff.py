#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#

import sys
import time
import argparse
import json
import zenoh
from zenoh import Reliability, Sample

import struct
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from sensor_msgs.msg import LaserScan

from VFF import VFF

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.clock import Clock
import rclpy

#include <tf2/LinearMath/Quaternion.h>
from geometry_msgs.msg import Quaternion

# --- Command line argument parsing --- --- --- --- --- ---

parser = argparse.ArgumentParser(
    prog='zp_lidar_vff',
    description='laser scan subscriber')
parser.add_argument('--mode', '-m', dest='mode',
                    choices=['peer', 'client'],
                    type=str,
                    help='The zenoh session mode.')
parser.add_argument('--connect', '-e', dest='connect',
                    metavar='ENDPOINT',
                    action='append',
                    type=str,
                    help='Endpoints to connect to.')
parser.add_argument('--listen', '-l', dest='listen',
                    metavar='ENDPOINT',
                    action='append',
                    type=str,
                    help='Endpoints to listen on.')
parser.add_argument('--config', '-c', dest='config',
                    metavar='FILE',
                    type=str,
                    help='A configuration file.')
parser.add_argument('--params-file', '-P', dest='params_file',
                    required=True,
                    metavar='FILE',
                    type=str,
                    help='A parameters json file.')

args = parser.parse_args()
conf = zenoh.Config.from_file(
    args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))

f = open(args.params_file)
params = json.load(f)["zenoh-python_lidar_vff"]
f.close()

sub_key = params["sub_key"]
pub_key = params["pub_key"]

# --- Some function definitions --- --- --- --- --- ---
import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(rpy : list):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  roll, pitch, yaw = rpy
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return Quaternion(x=qx, y=qy, z=qz, w=qw)



# --- Zenoh code --- --- --- --- --- ---

# initiate logging
zenoh.init_logger()

print("Opening session...")
session = zenoh.open(conf)

check_for_type_support(LaserScan)
check_for_type_support(Marker)
check_for_type_support(MarkerArray)

laser_scan_msg = LaserScan()
def listener(sample: Sample):
    global laser_scan_msg
    laser_scan_msg = _rclpy.rclpy_deserialize(sample.payload, LaserScan)
    #print(laser_scan_msg)
    #laser = VFF(laser_scan_msg)
    #print(laser_scan_msg)
    #print(laser_scan_msg.range_min)
    #print(laser_scan_msg.range_max)
    #print(laser_scan_msg.ranges)
    #ranges = list(laser_scan_msg.ranges)
    #print(len(laser_scan_msg.ranges))
    #print(list(laser_scan_msg.ranges)[180])


print("Declaring Subscriber on '{}'...".format(sub_key))
# WARNING, you MUST store the return value in order for the subscription to work!!
# This is because if you don't, the reference counter will reach 0 and the subscription
# will be immediately undeclared.
sub = session.declare_subscriber(sub_key, listener, reliability=Reliability.RELIABLE())

print(f"Declaring Publisher on '{pub_key}'...")
pub = session.declare_publisher(pub_key)


def get_marker(frame_id, position, orientation, scale, color, alpha=1.0, stamp=Clock().now().to_msg(), ns="marker_namespace", id=0, type=Marker.ARROW, action=Marker.ADD):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = id
    marker.type = type
    marker.action = action
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation = get_quaternion_from_euler(orientation)
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.a = alpha
    marker.color.r, marker.color.g, marker.color.b = color
    return marker



# --- main code and logic --- --- --- --- --- ---
def main():
    global laser_scan_msg
    z = 1.0
    w = 0.1
    inc = 0.1
    while True:
        virtual_filed_force = VFF(laser_scan_msg, objetive=[0, 0, 0])
        rep_theta, rep_r = virtual_filed_force.get_rep_force()

        marker_array = MarkerArray()
        marker_array.markers = []

        time.sleep(1e-2)

        #The main difference between them is that:
        #rclpy.time.Time() #this is the latest available transform in the buffer
        #Clock().now() # but this fetches the frame at the exact moment.
        #Thats the reason why with the last one doesn't work well.
        rep_force_marker = get_marker(id=0, frame_id="base_scan", ns="forces",
                                      stamp=rclpy.time.Time().to_msg(),
                                      type=Marker.ARROW, action=Marker.ADD,
                                      position=[0.0, 0.0, 0.0],
                                      orientation=[0.0, 0.0, rep_theta],
                                      scale=[rep_r/300, 0.05, 0.05],
                                      color=[1.0, 1.0, 0.0], alpha=0.5)
        marker_array.markers.append(rep_force_marker)

        atr_force_marker = get_marker(id=1, frame_id="base_scan", ns="forces",
                                      stamp=rclpy.time.Time().to_msg(),
                                      type=Marker.ARROW, action=Marker.ADD,
                                      position=[0.0, 0.0, 0.0],
                                      orientation=[0.0, 0.0, rep_theta],
                                      scale=[-rep_r/300, 0.05, 0.05],
                                      color=[0.0, 1.0, 1.0], alpha=0.5)
        marker_array.markers.append(atr_force_marker)
        
        """
        #Test to viualize markers in rviz
        bouncing_ball_marker = Marker()
        marker_array.markers = []
        bouncing_ball_marker.header.frame_id = "base_link"
        bouncing_ball_marker.header.stamp = Clock().now().to_msg()
        bouncing_ball_marker.ns = "my_namespace"
        bouncing_ball_marker.id = 0
        bouncing_ball_marker.type = Marker.SPHERE
        bouncing_ball_marker.action = Marker.ADD
        bouncing_ball_marker.pose.position.x = 1.0
        bouncing_ball_marker.pose.position.y = 1.0
        bouncing_ball_marker.pose.position.z = z
        bouncing_ball_marker.pose.orientation.x = 0.0
        bouncing_ball_marker.pose.orientation.y = 0.0
        bouncing_ball_marker.pose.orientation.z = 0.0
        bouncing_ball_marker.pose.orientation.w = 1.0
        bouncing_ball_marker.scale.x = w/2
        bouncing_ball_marker.scale.y = w/2
        bouncing_ball_marker.scale.z = min(z, 0.5)
        bouncing_ball_marker.color.a = 1.0
        bouncing_ball_marker.color.r = 0.0
        bouncing_ball_marker.color.g = 1.0
        bouncing_ball_marker.color.b = 0.0
        
        time.sleep(3e-2)
        if w >= 0.95:
            inc = -0.1
        if w < 0.11:
            inc = 0.1
        w += inc
        z -= inc

        #marker_array.markers.append(bouncing_ball_marker)
        """

        print("zenoh publishing marker")
        ser_msg = _rclpy.rclpy_serialize(marker_array, type(marker_array))
        pub.put(ser_msg)
        #for marker in marker_array.markers:
        #    ser_msg = _rclpy.rclpy_serialize(marker, type(marker))
        #    pub.put(ser_msg)
        

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Cleanup: note that even if you forget it, cleanup will happen automatically when 
        # the reference counter reaches 0
        #pub.undeclare()
        sub.undeclare()
        session.close()