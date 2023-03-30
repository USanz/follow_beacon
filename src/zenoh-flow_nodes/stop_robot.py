import zenoh

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support
from geometry_msgs.msg import Twist

session = zenoh.open(zenoh.Config())
pub = session.declare_publisher("rt/cmd_vel")

check_for_type_support(Twist)
twist_msg = Twist()
twist_msg.linear.x = 0.0
twist_msg.linear.y = 0.0
twist_msg.linear.z = 0.0
twist_msg.angular.x = 0.0
twist_msg.angular.y = 0.0
twist_msg.angular.z = 0.0

print("Stoping the motors")
ser_msg = _rclpy.rclpy_serialize(twist_msg, type(twist_msg))
pub.put(ser_msg)

pub.undeclare()
session.close()