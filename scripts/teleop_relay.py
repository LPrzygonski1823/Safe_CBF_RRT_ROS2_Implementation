"""Foxglove Teleop panel (geometry_msgs/Twist) -> twist_mux manual input (TwistStamped).

Panel publishes /manual/cmd_vel_unstamped; this node clamps it and republishes it on
/manual/cmd_vel. When the panel goes silent it sends a single zero command and stops
publishing, so it does not hold the mux lock against autonomous navigation.
"""
import math
import os

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


def clamp(value, limit):
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


class TeleopRelay(Node):
    def __init__(self):
        super().__init__('foxglove_teleop_relay')
        self.max_lin = self.declare_parameter(
            'max_linear', float(os.environ.get('TELEOP_MAX_LIN', '0.3'))).value
        self.max_ang = self.declare_parameter(
            'max_angular', float(os.environ.get('TELEOP_MAX_ANG', '1.0'))).value
        self.timeout = self.declare_parameter(
            'timeout', float(os.environ.get('TELEOP_TIMEOUT', '0.5'))).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.pub = self.create_publisher(TwistStamped, '/manual/cmd_vel', 1)
        self.create_subscription(Twist, '/manual/cmd_vel_unstamped', self.on_twist, 1)
        self.last_rx = None
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info(
            f'relay up: max_linear={self.max_lin} m/s, max_angular={self.max_ang} rad/s, '
            f'timeout={self.timeout} s')

    def publish(self, lin_x, lin_y, ang_z):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist.linear.x = lin_x
        out.twist.linear.y = lin_y
        out.twist.angular.z = ang_z
        self.pub.publish(out)

    def on_twist(self, msg):
        self.last_rx = self.get_clock().now()
        self.publish(clamp(msg.linear.x, self.max_lin),
                     clamp(msg.linear.y, self.max_lin),
                     clamp(msg.angular.z, self.max_ang))

    def watchdog(self):
        if self.last_rx is None:
            return
        age = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
        if age > self.timeout:
            self.publish(0.0, 0.0, 0.0)  # one explicit stop, then stay silent
            self.last_rx = None


def main():
    rclpy.init()
    node = TeleopRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()