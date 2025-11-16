#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/world/default/pose/info',
            self.tf_callback,
            10
        )

        self.get_logger().info('Pose to Odometry converter started (using TF)')

    def tf_callback(self, msg):
        # Find the base_link transform
        for transform in msg.transforms:
            if 'base_link' in transform.child_frame_id or transform.child_frame_id == 'eclipse_ackermann_bot':
                # Create odometry message
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'

                # Copy pose from transform
                odom.pose.pose.position.x = transform.transform.translation.x
                odom.pose.pose.position.y = transform.transform.translation.y
                odom.pose.pose.position.z = transform.transform.translation.z
                odom.pose.pose.orientation = transform.transform.rotation

                self.odom_pub.publish(odom)
                return


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
