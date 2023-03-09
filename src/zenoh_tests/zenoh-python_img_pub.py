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
import itertools
import json
import zenoh
from zenoh import config
import cv2, numpy as np

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(
    prog='z_pub',
    description='zenoh pub example')
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
parser.add_argument('--key', '-k', dest='key',
                    default='rt/camera/image_compressed',
                    type=str,
                    help='The key expression to publish onto.')
parser.add_argument('--value', '-v', dest='value',
                    default=cv2.imread('/home/usanz/zs_t3/t3_ws/src/follow_beacon/src/qr_tests/qr_code.png'),
                    type=np.ndarray,
                    help='The value to publish.')
parser.add_argument("--iter", dest="iter", type=int,
                    help="How many puts to perform")
parser.add_argument('--config', '-c', dest='config',
                    metavar='FILE',
                    type=str,
                    help='A configuration file.')
parser.add_argument('--rate', '-r', dest='img_rate',
                    type=int, default=2,
                    help='An image publishing rate (fps)')
parser.add_argument('--quality', '-q', dest='jpg_quality',
                    type=int, default=90,
                    help='An image quality jpg percentage.')
parser.add_argument('--width', '-x', dest='img_resize_width',
                    type=int, default=640,
                    help='An image width resize value.')
parser.add_argument('--height', '-y', dest='img_resize_height',
                    type=int, default=480,
                    help='An image height resize value.')

args = parser.parse_args()
conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
key = args.key
value = args.value
rate = args.img_rate
quality = args.jpg_quality
img_resize_size = (args.img_resize_width, args.img_resize_height)

# initiate logging
zenoh.init_logger()

print("Opening session...")
session = zenoh.open(conf)

print(f"Declaring Publisher on '{key}'...")
pub = session.declare_publisher(key)

print("Opening the camera...")
camera = cv2.VideoCapture(0)
if (not camera.isOpened()): 
  print("Error opening video stream or file")
for idx in itertools.count() if args.iter is None else range(args.iter):
    time.sleep(1.0/rate)

    frame_grabbed, frame = camera.read()
    if frame_grabbed:
        resized_frame = cv2.resize(frame, img_resize_size, interpolation = cv2.INTER_AREA)
        print(f"size: {resized_frame.size}, shape: {resized_frame.shape}")

        #cv2.imshow("window", resized_frame)
        #cv2.waitKey(10)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, encimg = cv2.imencode('.jpg', resized_frame, encode_param)
        print(type(encimg))
        print(f"size: {encimg.size}, shape: {encimg.shape}")

        print("Sending cam img...")
        pub.put(encimg.tobytes()) #only put the image

pub.undeclare()
session.close()