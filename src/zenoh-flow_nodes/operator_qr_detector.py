from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import cv2, numpy as np
from pyzbar.pyzbar import decode, ZBarSymbol
import struct

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from zenoh_nodes.QRCode import QRCode

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class OperatorQRDetector(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        self.input = inputs.get("Image", None)
        self.output = outputs.get("QR_Data", None)
        self.img_debug_output = outputs.get("Debug_Image", None)

        configuration = {} if configuration is None else configuration
        self.display_gui = bool(configuration.get("display_gui", False))
        self.detector = str(configuration.get("qr_code_detector", "zbar"))
        self.qr_data_to_track = str(configuration.get("qr_data_to_track", ""))

        check_for_type_support(Image)
        self.bridge = CvBridge()

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
                print("code/s found")
                for qr_code in qr_codes: #draw bounding boxes info decoded from QR codes:
                    dec_img = qr_code.draw_bbox(dec_img, (0, 255, 0), (0, 0, 255))
            img_msg = self.bridge.cv2_to_imgmsg(dec_img, encoding='rgb8')
            ser_msg = _rclpy.rclpy_serialize(img_msg, type(img_msg))
            await self.img_debug_output.send(ser_msg)

        #select the only biggest QR code matching:
        qr_code_to_track = get_biggest_qr_code_matching(qr_codes, self.qr_data_to_track)

        #the mesage is a list of floats (x_pos, y_pos, diag_size):
        qr_msg = [0.0, 0.0, -1.0]
        if qr_code_to_track != None:
            qr_msg = qr_code_to_track.get_centroid_rel()
            qr_msg.append(qr_code_to_track.get_diag_avg_size())
        
        #serialize ans send
        buf = struct.pack('%sf' % len(qr_msg), *qr_msg)
        await self.output.send(buf)
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

#def int_to_bytes(x: int) -> bytes:
#    return x.to_bytes((x.bit_length() + 7) // 8, "big")

#def int_from_bytes(xbytes: bytes) -> int:
#    return int.from_bytes(xbytes, "big")

#def str_to_bytes(string: str) -> bytes:
#    return bytes(string, 'utf-8')

def img_from_bytes(xbytes: bytes) -> np.ndarray:
    enc_img = np.frombuffer(xbytes, dtype=np.uint8)
    dec_img = cv2.imdecode(enc_img, cv2.IMREAD_COLOR)
    return dec_img


def register():
    return OperatorQRDetector
