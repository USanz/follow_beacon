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
from geometry_msgs.msg import Twist



# --- Command line argument parsing --- --- --- --- --- ---

parser = argparse.ArgumentParser(
    prog='zp_sink_motors',
    description='zenoh velocity publisher (it subscribes to the qr charasteristics and publishes velocities to the robot)')
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
params = json.load(f)["zenoh-python_sink_motors"]
f.close()

sub_key = params["sub_key"]
pub_key = params["pub_key"]
only_rotation_mode = params["only_rotation_mode"]
only_display_mode = params["only_display_mode"]
goal_x_dist = params["goal_x_dist"]
goal_pix_diag = params["goal_pix_diag"]
goal_x_threshold = params["goal_x_threshold"]
goal_pix_diag_threshold = params["goal_pix_diag_threshold"]
kp_v = params["kp_v"]
kp_w = params["kp_w"]
target_lost_timeout = params["target_lost_timeout"]



# --- Some function definitions --- --- --- --- --- ---

def get_time_ms():
    return time.time() * 1e3 # get the number of milliseconds after epoch.

def publish(cmd_vel_msg):
    ser_msg = _rclpy.rclpy_serialize(cmd_vel_msg, type(cmd_vel_msg))
    pub.put(ser_msg)

def get_twist_msg(init_vels = []):
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.0

    if len(init_vels) == 6: # unwrap
        twist_msg.linear.x, twist_msg.linear.y,
        twist_msg.linear.z, twist_msg.angular.x,
        twist_msg.angular.y, twist_msg.angular.z = init_vels

    return twist_msg

def stop_robot():
    cmd_vel_msg = get_twist_msg()
    publish(cmd_vel_msg)



# --- Zenoh code --- --- --- --- --- ---

# initiate logging
zenoh.init_logger()

print("Opening session...")
session = zenoh.open(conf)

check_for_type_support(Twist)

message = [0.0, 0.0, -1.0]
def listener(sample: Sample):
    global message
    #To receive the floats list:
    #We know for sure that we'll receive a list with the centroid coordinates (x, y)
    #and the diagonal size of the QR code, so the total lenght of the list is 3:
    array_length = 3
    message = struct.unpack('%sf' % array_length, sample.payload) # bytes to tuple
    
    #Other way is to receive the json format string:
    #buf = sample.payload.decode('utf-8') #bytes to str
    #print(json.loads(buf))
    
    #print(message)

print(f"Declaring Publisher on '{pub_key}'...")
pub = session.declare_publisher(pub_key)

print("Declaring Subscriber on '{}'...".format(sub_key))
# WARNING, you MUST store the return value in order for the subscription to work!!
# This is because if you don't, the reference counter will reach 0 and the subscription
# will be immediately undeclared.

target_lost_ts = get_time_ms()
sub = session.declare_subscriber(sub_key, listener, reliability=Reliability.RELIABLE())



# --- main code and logic --- --- --- --- --- ---

def main():
    global target_lost_ts

    while True:
        #print(message)
        qr_x, qr_y, qr_avg_diag_size = message
        
        cmd_vel_msg = get_twist_msg()

        error_x_dist = goal_x_dist - qr_x
        wanted_w = kp_w * error_x_dist #angular vel.
        error_pix_diag = goal_pix_diag - qr_avg_diag_size
        wanted_v = kp_v * error_pix_diag #linear vel.

        if not only_display_mode:
            if qr_avg_diag_size > 0:
                if (abs(error_pix_diag) >= goal_pix_diag_threshold\
                        and not only_rotation_mode):
                    cmd_vel_msg.linear.x = wanted_v
                if (abs(error_x_dist) >= goal_x_threshold):
                    cmd_vel_msg.angular.z = wanted_w
                target_lost_ts = get_time_ms() #reset time
            else:
                new_ts = get_time_ms()
                if (new_ts - target_lost_ts >= target_lost_timeout):
                    cmd_vel_msg.angular.z = 0.1

        print(f"Calculated vels [v, w]: [{wanted_v}, {wanted_w}] Sending vels [v, w]: [{cmd_vel_msg.linear.x}, {cmd_vel_msg.angular.z}]")

        publish(cmd_vel_msg)

        time.sleep(1e-2)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_robot()
        # Cleanup: note that even if you forget it, cleanup will happen automatically when 
        # the reference counter reaches 0
        pub.undeclare()
        sub.undeclare()
        session.close()