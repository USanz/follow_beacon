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
from geometry_msgs.msg import Quaternion
#from geometry_msgs.msg import TransformStamped
#from rclpy.clock import Clock

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from zenoh_nodes.VFF import VFF

#from rclpy.node import Node
#from tf2_ros import TransformBroadcaster
#import rclpy
#import tf2_ros
#from geometry_msgs.msg import TransformStamped
#from tf2_msgs.msg import TFMessage

#class FixedFrameBroadcaster(Node):
#
#   def __init__(self):
#       super().__init__('fixed_frame_tf2_broadcaster')
#       self.tf_broadcaster = TransformBroadcaster(self)
#       #self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
#
#   def broadcast_timer_callback(self):
#       t = TransformStamped()
#
#       t.header.stamp = self.get_clock().now().to_msg()
#       t.header.frame_id = 'base_scan'
#       t.child_frame_id = 'new_tf'
#       t.transform.translation.x = 0.0
#       t.transform.translation.y = 0.0
#       t.transform.translation.z = 1.0
#       t.transform.rotation.x = 0.0
#       t.transform.rotation.y = 0.0
#       t.transform.rotation.z = 0.0
#       t.transform.rotation.w = 1.0
#
#       self.tf_broadcaster.sendTransform(t)


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
        self.kp_r, self.kp_a = 1/1000, 50
        self.VFF = VFF(self.scan_msg, self.qr_msg, max_dist=1.0, kps=(self.kp_r, self.kp_a))
        self.pending = []

        #node = Node("nodo0")
        #node.__init__()
        #self.tf_broadcaster = TransformBroadcaster(node)
        #rclpy.init()
        #self.node = FixedFrameBroadcaster()

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

        self.VFF = VFF(self.scan_msg, self.qr_msg, max_dist=1.0, kps=(self.kp_r, self.kp_a))
        rep_f, atr_f, tot_f = self.VFF.get_forces()
        rep_theta, rep_r = rep_f
        atr_theta, atr_r = atr_f
        tot_theta, tot_r = tot_f

        tot_force_msg = [tot_theta, tot_r]
        buf = struct.pack('%sf' % len(tot_force_msg), *tot_force_msg)
        print(f"OPERATOR_LIDAR_VFF -> Sending force: {tot_force_msg}")
        await self.output_force.send(buf)

        marker_array = MarkerArray()
        marker_array.markers = []

        markers_pos = [0.0, 0.0, 0.1]
        rep_scale_factor, atr_scale_factor, tot_scale_factor = [1, 1, 1] #[1/300, 100, 1]
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

        #qr_code_tf = TransformStamped()
        #qr_code_tf.header.stamp = Clock().now().to_msg()
        #qr_code_tf.header.frame_id = 'base_scan'
        #qr_code_tf.child_frame_id = 'new_tf'
        #qr_code_tf.transform.translation.x = 0.0
        #qr_code_tf.transform.translation.y = 0.0
        #qr_code_tf.transform.translation.z = 1.0
        #qr_code_tf.transform.rotation = get_quaternion_from_euler([0.0, 0.0, 0.1])
        
        #ser_msg = _rclpy.rclpy_serialize(qr_code_tf, type(qr_code_tf))
        #await self.output_tf.send(ser_msg)
        
        #self.tf_broadcaster.sendTransform(qr_code_tf)
        
        #self.node.broadcast_timer_callback()
        
        #rclpy.spin_once(self.node)



        #TODO: read the TF somehow (maybe receiving directly from /rt/tf), the following code doesn't work:
        #lt = Duration()
        #lt.sec = 0
        #lt.nanosec = int(0.3e9) # 0.3s
        #buffer_core = tf2_ros.BufferCore(lt)
        #ts1 = TransformStamped()
        #ts1.header.stamp = rclpy.time.Time().to_msg()
        #ts1.header.frame_id = 'base_scan'
        #ts1.child_frame_id = 'qr_code'
        #ts1.transform.translation.x = 2.71828183
        #ts1.transform.rotation.w = 1.0
        #buffer_core.set_transform(ts1, "default_authority")
        


        #None of this options works, it says that:LookupException('\"qr_code\"
        #passed to lookupTransform argument target_frame does not exist. ')

        #a = buffer_core.lookup_transform_core('base_scan', 'qr_code', rclpy.time.Time())
        #a = buffer_core.lookup_transform_core('base_scan', 'qr_code', Clock().now())
        #a = buffer_core.lookup_transform_core('qr_code', 'base_scan', rclpy.time.Time())
        #a = buffer_core.lookup_transform_core('qr_code', 'base_scan', Clock().now())
        #print(a)

        time.sleep(1e-2)
        return None

    def finalize(self) -> None:
        #rclpy.shutdown()
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
