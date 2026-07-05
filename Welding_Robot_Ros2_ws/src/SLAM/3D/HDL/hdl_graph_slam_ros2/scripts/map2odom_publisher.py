#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Map2OdomPublisher(Node):
    def __init__(self):
        super().__init__('map2odom_publisher')

        # Parameters
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')

        self.default_map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.default_odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value

        # TF broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.odom_msg = None
        self.subscription = self.create_subscription(
            TransformStamped,
            '/hdl_graph_slam/odom2pub',
            self.callback,
            10)

        # Timer for publishing TF at 10Hz
        self.timer = self.create_timer(0.1, self.spin)

    def callback(self, odom_msg):
        self.odom_msg = odom_msg

    def spin(self):
        if self.odom_msg is None:
            # Publish identity transform while waiting for first odom message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.default_map_frame_id
            t.child_frame_id = self.default_odom_frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)
            return

        # Publish received transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_msg.header.frame_id
        t.child_frame_id = self.odom_msg.child_frame_id
        t.transform = self.odom_msg.transform
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Map2OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
