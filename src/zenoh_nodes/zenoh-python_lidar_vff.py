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
import rclpy

from rclpy.clock import Clock
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

lidar_sub_key = params["lidar_sub_key"]
centroid_sub_key = params["centroid_sub_key"]
markers_pub_key = params["markers_pub_key"]
total_force_pub_key = params["force_pub_key"]

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
def lidar_listener(sample: Sample):
    global laser_scan_msg
    laser_scan_msg = _rclpy.rclpy_deserialize(sample.payload, LaserScan)

centroid_msg = [0.0, 0.0, -1.0]
def centroid_listener(sample: Sample):
    global centroid_msg
    array_length = 3
    centroid_msg = struct.unpack('%sf' % array_length, sample.payload) # bytes to tuple

print("Declaring Subscriber on '{}'...".format(lidar_sub_key))
# WARNING, you MUST store the return value in order for the subscription to work!!
# This is because if you don't, the reference counter will reach 0 and the subscription
# will be immediately undeclared.
lidar_sub = session.declare_subscriber(lidar_sub_key, lidar_listener, reliability=Reliability.RELIABLE())

print("Declaring Subscriber on '{}'...".format(centroid_sub_key))
centroid_sub = session.declare_subscriber(centroid_sub_key, centroid_listener, reliability=Reliability.RELIABLE())

print(f"Declaring Publisher on '{markers_pub_key}'...")
markers_pub = session.declare_publisher(markers_pub_key)

print(f"Declaring Publisher on '{total_force_pub_key}'...")
total_force_pub = session.declare_publisher(total_force_pub_key)

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
        virtual_filed_force = VFF(laser_scan_msg, objetive=centroid_msg)
        kp_r, kp_a = 1/1000, 400
        rep_theta, rep_r = virtual_filed_force.get_rep_force()
        atr_theta, atr_r = virtual_filed_force.get_atr_force()
        tot_theta, tot_r = virtual_filed_force.get_tot_force(kp_r, kp_a)
        print("rep: ", rep_theta, kp_r * rep_r)
        print("atr: ", atr_theta, kp_a * atr_r)
        print("tot: ", tot_theta, tot_r)

        tot_force_msg = [tot_theta, tot_r]
        buf = struct.pack('%sf' % len(tot_force_msg), *tot_force_msg)
        print(f"Publishing: {tot_force_msg}")
        total_force_pub.put(buf)
        
        marker_array = MarkerArray()
        marker_array.markers = []

        #The main difference between them is that:
        #rclpy.time.Time() #this is the latest available transform in the buffer
        #from rclpy.clock import Clock
        #Clock().now() # but this fetches the frame at the exact moment.
        #Thats the reason why with the last one doesn't work well.
        markers_pos = [0.0, 0.0, 0.1]
        rep_force_marker = get_marker(id=0, frame_id="base_scan", ns="forces",
                                      stamp=rclpy.time.Time().to_msg(),
                                      type=Marker.ARROW, action=Marker.ADD,
                                      position=markers_pos,
                                      orientation=[0.0, 0.0, rep_theta],
                                      scale=[rep_r/300, 0.05, 0.05],
                                      color=[1.0, 0.5, 0.0], alpha=0.5)
        atr_force_marker = get_marker(id=1, frame_id="base_scan", ns="forces",
                                      stamp=rclpy.time.Time().to_msg(),
                                      type=Marker.ARROW, action=Marker.ADD,
                                      position=markers_pos,
                                      orientation=[0.0, 0.0, atr_theta],
                                      scale=[atr_r*100, 0.05, 0.05],
                                      color=[0.5, 1.0, 0.0], alpha=0.5)
        tot_force_marker = get_marker(id=2, frame_id="base_scan", ns="forces",
                                      stamp=rclpy.time.Time().to_msg(),
                                      type=Marker.ARROW, action=Marker.ADD,
                                      position=markers_pos,
                                      orientation=[0.0, 0.0, tot_theta],
                                      scale=[tot_r, 0.05, 0.05],
                                      color=[0.0, 1.0, 1.0], alpha=0.5)
        
        marker_array.markers.append(rep_force_marker)
        marker_array.markers.append(atr_force_marker)
        marker_array.markers.append(tot_force_marker)
        
        print("zenoh publishing marker")
        ser_msg = _rclpy.rclpy_serialize(marker_array, type(marker_array))
        markers_pub.put(ser_msg)

        time.sleep(1e-2)
        

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Cleanup: note that even if you forget it, cleanup will happen automatically when 
        # the reference counter reaches 0
        markers_pub.undeclare()
        total_force_pub.undeclare()
        lidar_sub.undeclare()
        centroid_sub.undeclare()
        session.close()