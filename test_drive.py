#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TestDrive(Node):
    def __init__(self):
        super().__init__('test_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_received = False

    def odom_callback(self, msg):
        self.odom_received = True
        print(f"Odom - Position: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}")
        print(f"       Velocity: linear={msg.twist.twist.linear.x:.3f}, angular={msg.twist.twist.angular.z:.3f}")

def main():
    rclpy.init()
    node = TestDrive()

    # Give time for connections
    import time
    time.sleep(1)

    # Create and send a twist message
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0

    print(f"Publishing: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")
    print("Waiting for odometry feedback...")

    for i in range(50):
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.odom_received:
        print("\nWARNING: No odometry received! The diff drive plugin might not be loaded.")
    else:
        print("\nDone!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
