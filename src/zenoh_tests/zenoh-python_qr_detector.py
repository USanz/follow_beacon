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
import struct
from QRCode import QRCode


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
                    default='rt/camera/image_compressed',
                    type=str,
                    help='The key expression to subscribe to receive the image from.')
parser.add_argument('--pub_key', '-p', dest='pub_key',
                    default='rt/centroid/rel_pos',
                    type=str,
                    help="The key expression to publish the QR code's centroid relative coordinates.")
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

def get_biggest_qr_code(qr_codes):
    if len(qr_codes) == 0:
        return None
    biggest_code = qr_codes[0]
    for qr_code in qr_codes[1:]:
        if qr_code > biggest_code:
            biggest_code = qr_code
    return biggest_code


def listener(sample: Sample):
    encimg = np.frombuffer(sample.payload, dtype=np.uint8)
    decimg = cv2.imdecode(encimg, 1)
    print(f"received image of size {decimg.size} and shape: {decimg.shape}")
    img_height, img_width, img_channels = decimg.shape
    code_found = False
    qr_codes = list()
    if detector == "opencv":
        qcd = cv2.QRCodeDetector() # openCV QR code detector and decoder
        code_found, decoded_info, qr_codes_points, straight_qrcode = qcd.detectAndDecodeMulti(decimg)
        #code_found, points = qcd.detectMulti(decimg) # detect only.
        if code_found:
            for points, data in zip(qr_codes_points, decoded_info):
                qr_codes.append(QRCode(points, data, [img_width, img_height]))
    elif detector == "zbar":
        decoded_objects = decode(decimg, symbols=[ZBarSymbol.QRCODE]) # Zbar QR code detector and decoder
        code_found = len(decoded_objects) > 0
        if code_found:
            for obj_decoded in decoded_objects:
                points = np.array(obj_decoded.polygon, np.int32)
                data = obj_decoded.data.decode('utf-8')
                qr_codes.append(QRCode(points, data, [img_width, img_height]))

    #visualization sometimes crashes when it has to show much data (higher quality or higher image rate (fps)):
    if display_gui:
        if code_found:
            print("code/s found")
            for qr_code in qr_codes: #draw bounding boxes info decoded from QR codes:
                decimg = qr_code.draw_bbox(decimg, (0, 255, 0), (0, 0, 255))
        cv2.imshow("Zenoh python QR code detector", decimg)
        cv2.waitKey(10)

    qr_code_to_track = get_biggest_qr_code(qr_codes)

    #Sending a list of floats serialized:
    qr_msg = [0.0, 0.0, -1.0]
    if qr_code_to_track != None:
        qr_msg = qr_code_to_track.get_centroid_rel()
        qr_msg.append(qr_code_to_track.get_diag_avg_size())
    
    buf = struct.pack('%sf' % len(qr_msg), *qr_msg)
    print(f"Publishing: {qr_msg}")
    pub.put(buf)

    #Other way is sending a json string format serialized:
    #message_dict = {"coords": [0.25534, -0.22145],
    #                "size": 13.2454}
    #buf = json.dumps(message_dict) #dict to str
    #print(f"Publishing: {buf}")
    #pub.put(buf)



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