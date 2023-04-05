from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import cv2, numpy as np
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode, ZBarSymbol
import struct

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from sensor_msgs.msg import Image

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from zenoh_nodes.QRCode import QRCode

from builtin_interfaces.msg import Duration
from rclpy.clock import Clock
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Quaternion


class OperatorQRDetector(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        self.input = inputs.get("Image", None)
        self.output_qr = outputs.get("QR_Data", None)
        self.output_debug_img = outputs.get("DebugImage", None)
        self.output_tf = outputs.get("TF", None)

        configuration = {} if configuration is None else configuration
        self.display_gui = bool(configuration.get("display_gui", False))
        self.detector = str(configuration.get("qr_code_detector", "zbar"))
        self.qr_data_to_track = str(configuration.get("qr_data_to_track", ""))

        self.bridge = CvBridge()
        check_for_type_support(Image)
        check_for_type_support(TransformStamped)
        check_for_type_support(TFMessage)

    async def iteration(self) -> None:
        # in order to wait on multiple input streams use:
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.gather
        # or
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.wait
        data_msg = await self.input.recv()
        dec_img = img_from_bytes(data_msg.data)
        
        #detect the QR codes:
        img_height, img_width, img_channels = dec_img.shape
        code_found = False
        qr_codes = list()
        if self.detector == "opencv":
            qcd = cv2.QRCodeDetector() # openCV QR code detector and decoder
            code_found, decoded_info, qr_codes_points, straight_qrcode = qcd.detectAndDecodeMulti(dec_img)
            if code_found:
                for points, data in zip(qr_codes_points, decoded_info):
                    qr_codes.append(QRCode(points, data, [img_width, img_height]))
        elif self.detector == "zbar":
            decoded_objects = decode(dec_img, symbols=[ZBarSymbol.QRCODE]) # Zbar QR code detector and decoder
            code_found = len(decoded_objects) > 0
            if code_found:
                for obj_decoded in decoded_objects:
                    points = np.array(obj_decoded.polygon, np.int32)
                    data = obj_decoded.data.decode('utf-8')
                    qr_codes.append(QRCode(points, data, [img_width, img_height]))

        if self.display_gui:
            if code_found:
                for qr_code in qr_codes: #draw bounding boxes info decoded from QR codes:
                    dec_img = qr_code.draw_bbox(dec_img, (0, 255, 0), (0, 0, 255))
            #Send the uncompressed and edited img to debug:
            scale = 0.5 #reduced 50% to avoid computing.
            new_size = (int(dec_img.shape[1] * scale), int(dec_img.shape[0] * scale))
            dec_img = cv2.resize(dec_img, new_size, interpolation=cv2.INTER_AREA)
            img_msg = self.bridge.cv2_to_imgmsg(dec_img)
            ser_msg = _rclpy.rclpy_serialize(img_msg, type(img_msg))
            await self.output_debug_img.send(ser_msg)

        #select the only biggest QR code matching:
        qr_code_to_track = get_biggest_qr_code_matching(qr_codes, self.qr_data_to_track)

        #the mesage is a list of floats (x_pos, y_pos, diag_size):
        qr_msg = [0.0, 0.0, -1.0]
        if qr_code_to_track != None:
            qr_msg[0], qr_msg[1] = qr_code_to_track.get_centroid_rel()
            qr_msg[2] = qr_code_to_track.get_diag_avg_size()
            #Publish the qr_tf:
            #TODO: have into account the camera overture in the maths
            #TODO: modify the tf lifetime if possible to ~10s
            scale_factor = 1/370
            lt = Duration()
            lt.sec = 3
            lt.nanosec = 0
            tf = TransformStamped()
            tf.header.stamp = Clock().now().to_msg()
            tf.header.frame_id = 'base_scan'
            tf.child_frame_id = 'qr_code'
            tf.transform.translation.x = (600 - qr_msg[2])*scale_factor
            tf.transform.translation.y = qr_msg[0]
            tf.transform.translation.z = 0.5-qr_msg[1]
            tf.transform.rotation = get_quaternion_from_euler([0.0, 0.0, 0.0])
            
            tf_msg = TFMessage()
            tf_msg.transforms.append(tf)
            ser_tf_msg = _rclpy.rclpy_serialize(tf_msg, type(tf_msg))
            await self.output_tf.send(ser_tf_msg)
        
        #serialize ans send
        buf = struct.pack('%sf' % len(qr_msg), *qr_msg)
        await self.output_qr.send(buf)
        print(f"OPERATOR_QR_DETECTOR -> Sending QR data: {qr_msg}")

        return None

    def finalize(self) -> None:
        return None


def get_biggest_qr_code_matching(qr_codes, data):
    if len(qr_codes) == 0:
        return None
    biggest_code = None
    for qr_code in qr_codes:
        if (biggest_code == None or qr_code > biggest_code) and qr_code.data_matches(data):
            biggest_code = qr_code
    return biggest_code

def img_from_bytes(xbytes: bytes) -> np.ndarray:
    enc_img = np.frombuffer(xbytes, dtype=np.uint8)
    dec_img = cv2.imdecode(enc_img, cv2.IMREAD_COLOR)
    return dec_img

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


def register():
    return OperatorQRDetector
