from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import numpy as np

import json, time

import asyncio

import zenoh
from zenoh import Reliability, Sample

import struct
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from zenoh_nodes.VFF import VFF

class OperatorLidarVFF(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        self.input_qr = inputs.get("QR_Data", None)
        self.input_scan = inputs.get("Scan", None)
        self.output_force = outputs.get("Force", None)
        self.output_markers = outputs.get("Markers", None)

        check_for_type_support(LaserScan)
        check_for_type_support(Marker)
        check_for_type_support(MarkerArray)

        configuration = {} if configuration is None else configuration

        self.scan_msg = LaserScan()
        self.qr_msg = [0.0, 0.0, -1.0]
        self.VFF = VFF(self.scan_msg, self.qr_msg)
        self.pending = []

    async def wait_qr(self):
        data_msg = await self.input_qr.recv()
        return ("QR", data_msg)

    async def wait_scan(self):
        data_msg = await self.input_scan.recv()
        return ("Scan", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "QR" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_qr(), name="QR")
            )

        if not any(t.get_name() == "Scan" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_scan(), name="Scan")
            )
        return task_list

    async def iteration(self) -> None:
        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )

        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()
            if who == "QR":
                #print("qr received")
                array_length = 3
                self.qr_msg = struct.unpack('%sf' % array_length, data_msg.data) # bytes to tuple
            elif who == "Scan":
                #print("scan received")
                self.scan_msg = _rclpy.rclpy_deserialize(data_msg.data, LaserScan)

        self.VFF = VFF(self.scan_msg, objetive=self.qr_msg)
        kp_r, kp_a = 1/1000, 400
        rep_theta, rep_r = self.VFF.get_rep_force()
        atr_theta, atr_r = self.VFF.get_atr_force()
        tot_theta, tot_r = self.VFF.get_tot_force(kp_r, kp_a)
        #print("rep: ", rep_theta, kp_r * rep_r)
        #print("atr: ", atr_theta, kp_a * atr_r)
        #print("tot: ", tot_theta, tot_r)

        tot_force_msg = [tot_theta, tot_r]
        buf = struct.pack('%sf' % len(tot_force_msg), *tot_force_msg)
        print(f"OPERATOR_LIDAR_VFF -> Sending force: {tot_force_msg}")
        await self.output_force.send(buf)

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
            ser_msg = _rclpy.rclpy_serialize(marker_array, type(marker_array))
            await self.output_markers.send(ser_msg)

        time.sleep(1e-2)
        return None

    def finalize(self) -> None:
        return None


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


def register():
    return OperatorLidarVFF
