from zenoh_flow.interfaces import Sink
from zenoh_flow import Input
from zenoh_flow.types import Context
from typing import Dict, Any
import asyncio
import struct, time, json

import zenoh

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from geometry_msgs.msg import Twist


class SinkMotors(Sink):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
    ):
        self.input = inputs.get("QR_Data", None)

        configuration = {} if configuration is None else configuration
        
        self.only_rotation_mode = bool(configuration.get("only_rotation_mode", False))
        self.only_display_mode = bool(configuration.get("only_display_mode", False))
        
        self.goal_x_dist = float(configuration.get("goal_x_dist", 0.0))
        self.goal_pix_diag = float(configuration.get("goal_pix_diag", 160.0))
        self.goal_pix_diag_threshold = float(configuration.get("goal_pix_diag_threshold", 0.05))
        self.goal_x_threshold = float(configuration.get("goal_x_threshold", 0.1))
        self.kp_v = float(configuration.get("kp_v", 0.35))
        self.kp_w = float(configuration.get("kp_w", 0.005))

        self.target_lost_timeout = int(configuration.get("target_lost_timeout", 3000))
        self.target_lost_ts = get_time_ms()

        zenoh_config_file = str(configuration.get("zenoh_config_file", ""))
        mode = str(configuration.get("mode", ""))
        connect = str(configuration.get("connect", ""))
        listen = str(configuration.get("listen", ""))
        conf = zenoh.Config.from_file(
            zenoh_config_file) if zenoh_config_file != "" else zenoh.Config()
        if mode != "":
            conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(mode))
        if connect != "":
            conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(connect))
        if listen != "":
            conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(listen))
        self.session = zenoh.open(conf)
        
        pub_key = str(configuration.get("pub_key", "rt/cmd_vel"))
        self.pub = self.session.declare_publisher(pub_key)
        
        check_for_type_support(Twist)
    
    async def iteration(self) -> None:
        data_msg = await self.input.recv()
        array_length = 3
        message = struct.unpack('%sf' % array_length, data_msg.data) # bytes to tuple
        print(f"Received {message}")

        qr_x, qr_y, qr_avg_diag_size = message
        cmd_vel_msg = get_twist_msg()

        error_x_dist = self.goal_x_dist - qr_x
        wanted_w = self.kp_w * error_x_dist #angular vel.
        error_pix_diag = self.goal_pix_diag - qr_avg_diag_size
        wanted_v = self.kp_v * error_pix_diag #linear vel.
        
        if not self.only_display_mode:
            if qr_avg_diag_size > 0:
                if (abs(error_pix_diag) >= self.goal_pix_diag_threshold\
                        and not self.only_rotation_mode):
                    cmd_vel_msg.linear.x = wanted_v
                if (abs(error_x_dist) >= self.goal_x_threshold):
                    cmd_vel_msg.angular.z = wanted_w
                self.target_lost_ts = get_time_ms() #reset time
            else:
                new_ts = get_time_ms()
                if (new_ts - self.target_lost_ts >= self.target_lost_timeout):
                    cmd_vel_msg.angular.z = 0.1

        print(f"Calculated vels [v, w]: [{wanted_v}, {wanted_w}] Sending vels [v, w]: [{cmd_vel_msg.linear.x}, {cmd_vel_msg.angular.z}]")

        publish(self.pub, cmd_vel_msg)

        time.sleep(1e-2)


    def finalize(self): #not working for now
        stop_robot(self.pub)
        self.pub.undeclare()
        self.session.close()
        return None


def get_time_ms():
    return time.time() * 1e3 # get the number of milliseconds after epoch.

def publish(pub, cmd_vel_msg):
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

def stop_robot(pub):
    cmd_vel_msg = get_twist_msg()
    print("stoping robot...")
    publish(pub, cmd_vel_msg)

def register():
    return SinkMotors
