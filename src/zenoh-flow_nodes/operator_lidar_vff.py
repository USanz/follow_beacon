from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any
import numpy as np

import time

import asyncio

import struct
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from rclpy.clock import Clock
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, Point
from tf2_msgs.msg import TFMessage

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from zenoh_nodes.VFF import VFF

import tf2_ros, rclpy

class OperatorLidarVFF(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        self.input_tf = inputs.get("TF", None)
        self.input_tf_static = inputs.get("TF_static", None)
        self.input_scan = inputs.get("Scan", None)
        self.output_force = outputs.get("Force", None)
        self.output_markers = outputs.get("Markers", None)

        self.lidar_max_radius = float(configuration.get("lidar_max_radius", 1.0))
        self.kp_r = float(configuration.get("kp_r", 1 / 1000))
        self.kp_a = float(configuration.get("kp_a", 1))

        check_for_type_support(LaserScan)
        check_for_type_support(Marker)
        check_for_type_support(MarkerArray)
        check_for_type_support(TFMessage)

        configuration = {} if configuration is None else configuration

        self.scan_msg = LaserScan()
        self.tf_msg = TFMessage()
        self.tf_pos = [0.0, 0.0, -1.0]
        self.last_qr_pos = self.tf_pos
        self.VFF = VFF(self.scan_msg,
                       self.tf_pos,
                       max_dist=self.lidar_max_radius,
                       kps=(self.kp_r, self.kp_a))
        self.pending = []

        lt = Duration()
        lt.sec = 10
        lt.nanosec = 0
        self.buffer_core = tf2_ros.BufferCore(lt)

    async def wait_qr_tf(self):
        data_msg = await self.input_tf.recv()
        return ("TF", data_msg)

    async def wait_qr_tf_static(self):
        data_msg = await self.input_tf_static.recv()
        return ("TF_static", data_msg)

    async def wait_scan(self):
        data_msg = await self.input_scan.recv()
        return ("Scan", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "TF" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_qr_tf(), name="TF")
            )
        if not any(t.get_name() == "TF_static" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_qr_tf_static(), name="TF_static")
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

        tf_names = ["odom->base_footprint",
                    "base_footprint->base_link",
                    "base_link->base_scan",
                    "odom->qr_code_from_odom"]
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()
            if who in ["TF", "TF_static"]:
                self.tf_msg = _rclpy.rclpy_deserialize(data_msg.data, TFMessage)
                for tf in self.tf_msg.transforms:
                    tf_name = tf.header.frame_id + "->" + tf.child_frame_id
                    if tf_name in tf_names:
                        #tf.header.stamp = Clock().now().to_msg()
                        self.buffer_core.set_transform(tf, "default_authority")

                    #if tf_name == "base_scan->qr_code":
                    #    print("RELATIVE TO BASE_SCAN")
                    #    
                    #    #This updates but so slowly so it results on very high elapsed times:
                    #    ts_tf_ns = tf.header.stamp.sec * 1e9 + tf.header.stamp.nanosec
                    #    ts_now = Clock().now().nanoseconds
                    #    elapsed_total = ts_now - ts_tf_ns
                    #    print("time elsapsed in s:", elapsed_total*1e-9) #why the time is so high???

                    #    if elapsed_total < int(0.1e9): #1e9 ns = 1 s
                    #        self.tf_pos = [tf.transform.translation.x,
                    #                       tf.transform.translation.y,
                    #                       tf.transform.translation.z]
                    #        break

                    if tf_name == "odom->qr_code_from_odom":
                        print("ABSOLUTE (RELATIVE TO ODOM)")
                        try:
                            new_tf = self.buffer_core.lookup_transform_core('base_scan', 'qr_code_from_odom', rclpy.time.Time())
                            self.buffer_core.set_transform(new_tf, "t3_usanz_authority")
                            ts_tf_ns = new_tf.header.stamp.sec * 1e9 + new_tf.header.stamp.nanosec
                            ts_now = Clock().now().nanoseconds
                            elapsed_total = ts_now - ts_tf_ns
                            #print("time elsapsed in s:", elapsed_total*1e-9)

                            self.tf_pos = [new_tf.transform.translation.x,
                                           new_tf.transform.translation.y,
                                           new_tf.transform.translation.z]
                            #They are the same exact coords as in the operator qr detector node.
                            print("operator lidar qr tf from base_scan:", self.tf_pos)
                            self.last_qr_pos = self.tf_pos
                        except Exception as e:
                            pass

            elif who == "Scan":
                self.scan_msg = _rclpy.rclpy_deserialize(data_msg.data, LaserScan)

        self.VFF.set_lidar(self.scan_msg, max_dist=self.lidar_max_radius)
        self.VFF.set_target(self.tf_pos)
        rep_f, atr_f, tot_f = self.VFF.get_forces()
        rep_theta, rep_r = rep_f
        atr_theta, atr_r = atr_f
        tot_theta, tot_r = tot_f

        tot_force_msg = list(tot_f)
        buf = struct.pack('%sf' % len(tot_force_msg), *tot_force_msg)
        #print(f"OPERATOR_LIDAR_VFF -> Sending force: {tot_force_msg}")
        await self.output_force.send(buf)

        marker_array = MarkerArray()
        marker_array.markers = []

        markers_pos = [0.0, 0.0, 0.1]
        lt = Duration()
        lt.sec = 0
        lt.nanosec = int(0.3e9) # 0.3s
        rep_force_dict = {"id":0, "ns":"repulsion force", "type":Marker.ARROW,
                          "position":markers_pos, "frame_locked":False,
                          "orientation":[0.0, 0.0, rep_theta],
                          "scale":[rep_r, 0.05, 0.05],
                          "color_rgba":[1.0, 0.5, 0.0, 0.75], "lifetime":lt}
        atr_force_dict = {"id":1, "ns":"attraction force", "type":Marker.ARROW,
                          "position":markers_pos, "frame_locked":False,
                          "orientation":[0.0, 0.0, atr_theta],
                          "scale":[atr_r, 0.05, 0.05],
                          "color_rgba":[0.5, 1.0, 0.0, 0.75], "lifetime":lt}
        tot_force_dict = {"id":2, "ns":"total force", "type":Marker.ARROW,
                          "position":markers_pos, "frame_locked":False,
                          "orientation":[0.0, 0.0, tot_theta],
                          "scale":[tot_r, 0.05, 0.05],
                          "color_rgba":[0.0, 1.0, 1.0, 0.75], "lifetime":lt}
        for force_dict in [rep_force_dict, atr_force_dict, tot_force_dict]:
            if force_dict["scale"][0] > 0.0:
                marker_array.markers.append(get_marker(force_dict))

        #draw circle around the lidar with the min and max ranges:
        for i, range_val in enumerate([self.lidar_max_radius,
                                       self.scan_msg.range_min]):
            lidar_range_marker = circunf_markers(origin_frame_id="base_scan",
                                                    radius=range_val,
                                                    points_num=15, id=3+i)
            marker_array.markers.append(lidar_range_marker)

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

def get_marker(def_dict):
    marker = Marker()
    #The main difference between them is that:
    #marker.header.stamp = rclpy.time.Time().to_msg() #this is the latest available transform in the buffer
    #marker.header.stamp = Clock().now().to_msg()# but this fetches the frame at the exact moment ((from rclpy.clock import Clock).
    marker.header.frame_id = "base_scan"
    marker.ns = def_dict["ns"]
    marker.id = def_dict["id"]
    marker.frame_locked=def_dict["frame_locked"]
    marker.type = def_dict["type"]
    marker.action = Marker.ADD
    marker.lifetime = def_dict["lifetime"]
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = def_dict["position"]
    marker.pose.orientation = get_quaternion_from_euler(def_dict["orientation"])
    marker.scale.x, marker.scale.y, marker.scale.z = def_dict["scale"]
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = def_dict["color_rgba"]
    return marker

def circunf_markers(origin_frame_id: str, radius: float, points_num: int, id: int):
    lt = Duration()
    lt.sec = 1
    lt.nanosec = int(0.3e9) # 0.3s
    marker_dict = {"id":id, "ns":"lidar ranges", "type":Marker.LINE_STRIP,
                    "position":[0.0, 0.0, 0.0], "orientation":[0.0, 0.0, 0.0],
                    "scale":[0.01, 0.0, 0.0], "frame_locked":False,
                    "color_rgba":[1.0, 0.0, 0.0, 0.5], "lifetime":lt}
    marker = get_marker(marker_dict)
    angle_step = 360 / points_num
    for i in range(points_num):
        point = Point()
        point.x, point.y = polar2cart(i * angle_step, radius)
        point.z = 0.0
        marker.points.append(point)
    marker.points.append(marker.points[0]) # to close the circle
    return marker

def polar2cart(theta, r):
    x = r * np.cos(np.deg2rad(theta))
    y = r * np.sin(np.deg2rad(theta))
    return (x, y)


def register():
    return OperatorLidarVFF
