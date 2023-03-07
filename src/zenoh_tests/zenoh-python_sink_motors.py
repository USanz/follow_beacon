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
from datetime import datetime
import argparse
import json
import zenoh
from zenoh import Reliability, Sample
import cv2, numpy as np
from PIL import Image
import pyqrcode
from pyzbar.pyzbar import decode, ZBarSymbol
import json, struct


# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(
    prog='z_sub',
    description='zenoh sub example')
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
parser.add_argument('--sub_key', '-s', dest='sub_key',
                    default='rt/centroid/rel_pos',
                    type=str,
                    help='The key expression to subscribe to receive the QR code coordinates.')
parser.add_argument('--pub_key', '-p', dest='pub_key',
                    default='rt/cmd_vel',
                    type=str,
                    help="The key expression to publish the commanded velocities to the robot.")
parser.add_argument('--config', '-c', dest='config',
                    metavar='FILE',
                    type=str,
                    help='A configuration file.')
parser.add_argument('--detector', '-d', dest='qr_code_detector',
                    type=str, default='opencv',
                    choices=['opencv', 'zbar'],
                    help='A qr code detector library.')
parser.add_argument('--gui', '-g', dest='display_gui',
                    type=str, default='false',
                    choices=['false', 'true'],
                    help='A flag to display GUI.')

args = parser.parse_args()
conf = zenoh.Config.from_file(
    args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
sub_key = args.sub_key
pub_key = args.pub_key
detector = args.qr_code_detector
display_gui = args.display_gui == 'true'
# Zenoh code  --- --- --- --- --- --- --- --- --- --- ---



# initiate logging
zenoh.init_logger()

print("Opening session...")
session = zenoh.open(conf)

def listener(sample: Sample):
    #To receive the floats list:
    #We know for sure that we'll receive a list with the centroid coordinates (x, y)
    #and the diagonal size of the QR code, so the total lenght of the list is 3:
    array_length = 3
    message = struct.unpack('%sf' % array_length, sample.payload) # bytes to tuple
    
    #Other way is to receive the json format string:
    #buf = sample.payload.decode('utf-8') #bytes to str
    #print(json.loads(buf))
    
    print(message)
    qr_x = message[0]
    qr_y = message[1]
    qr_avg_diag_size = message[2]

    #TODO: publish a ROS2 Twist message with the velocities in /rt/cmd_vel


print(f"Declaring Publisher on '{pub_key}'...")
pub = session.declare_publisher(pub_key)

print("Declaring Subscriber on '{}'...".format(sub_key))
# WARNING, you MUST store the return value in order for the subscription to work!!
# This is because if you don't, the reference counter will reach 0 and the subscription
# will be immediately undeclared.
sub = session.declare_subscriber(sub_key, listener, reliability=Reliability.RELIABLE())


print("Enter 'q' to quit...")
c = '\0'
while c != 'q':
    c = sys.stdin.read(1)
    if c == '':
        time.sleep(1)

# Cleanup: note that even if you forget it, cleanup will happen automatically when 
# the reference counter reaches 0
pub.undeclare()
sub.undeclare()
session.close()