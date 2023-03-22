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
from builtin_interfaces.msg import Duration
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

def get_marker(force_def_dict):
    marker = Marker()
    #The main difference between them is that:
    #marker.header.stamp = rclpy.time.Time().to_msg() #this is the latest available transform in the buffer
    #marker.header.stamp = Clock().now().to_msg()# but this fetches the frame at the exact moment.
    marker.header.frame_id = "base_scan"
    marker.ns = force_def_dict["ns"]
    marker.id = force_def_dict["id"]
    marker.type = force_def_dict["type"]
    marker.action = Marker.ADD
    marker.lifetime = force_def_dict["lifetime"]
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = force_def_dict["position"]
    marker.pose.orientation = get_quaternion_from_euler(force_def_dict["orientation"])
    marker.scale.x, marker.scale.y, marker.scale.z = force_def_dict["scale"]
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = force_def_dict["color_rgba"]
    return marker



# --- main code and logic --- --- --- --- --- ---
def main():
    global laser_scan_msg
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

        markers_pos = [0.0, 0.0, 0.1]
        rep_scale_factor, atr_scale_factor, tot_scale_factor = [1/300, 100, 1]
        lt = Duration()
        lt.sec = 0
        lt.nanosec = int(0.3e9) # 0.3s
        rep_force_dict = {"id":0, "ns":"repulsion force", "type":Marker.ARROW,
                          "position":markers_pos, "orientation":[0.0, 0.0, rep_theta],
                          "scale":[rep_r*rep_scale_factor, 0.05, 0.05],
                          "color_rgba":[1.0, 0.5, 0.0, 0.5], "lifetime":lt}
        atr_force_dict = {"id":1, "ns":"attraction force", "type":Marker.ARROW,
                          "position":markers_pos, "orientation":[0.0, 0.0, atr_theta],
                          "scale":[atr_r*atr_scale_factor, 0.05, 0.05],
                          "color_rgba":[0.5, 1.0, 0.0, 0.5], "lifetime":lt}
        tot_force_dict = {"id":2, "ns":"total force", "type":Marker.ARROW,
                          "position":markers_pos, "orientation":[0.0, 0.0, tot_theta],
                          "scale":[tot_r*tot_scale_factor, 0.05, 0.05],
                          "color_rgba":[0.0, 1.0, 1.0, 0.5], "lifetime":lt}
        for force_dict in [rep_force_dict, atr_force_dict, tot_force_dict]:
            if force_dict["scale"][0] > 0.0:
                marker_array.markers.append(get_marker(force_dict))

        if marker_array.markers:
            print("zenoh publishing markers")
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