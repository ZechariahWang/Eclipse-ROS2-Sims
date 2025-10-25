#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.5
        self.get_logger().info('Drive Forward node started - Robot moving forward at 0.5 m/s')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity  # Move forward
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0  # No rotation

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    drive_forward_node = DriveForward()

    try:
        rclpy.spin(drive_forward_node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    drive_forward_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
