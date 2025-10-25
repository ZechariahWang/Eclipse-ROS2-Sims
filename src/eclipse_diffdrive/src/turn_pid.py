#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


class TurnPID(Node):
    def __init__(self):
        super().__init__('turn_pid')

        self.declare_parameter('target_angle', 90.0)  # Target angle in degrees
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('angle_tolerance', 2.0)  # Degrees

        self.target_angle_deg = self.get_parameter('target_angle').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.angle_tolerance = math.radians(self.get_parameter('angle_tolerance').value)

        self.target_angle = self.normalize_angle(math.radians(self.target_angle_deg))
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.max_angular_speed = 1.5  
        self.min_angular_speed = 0.1  
        self.current_yaw = None
        self.initial_yaw = None
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # State Management
        self.goal_reached = False

        # Control Loop Timer (20 Hz for smoother control)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Turn PID Controller Started')
        self.get_logger().info(f'Target angle: {self.target_angle_deg}° ({self.target_angle:.2f} rad)')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.current_yaw = yaw
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info(f'Initial heading: {math.degrees(yaw):.2f}°')

    def control_loop(self):
        if self.current_yaw is None:
            self.get_logger().warn('No odometry data available', throttle_duration_sec=2.0)
            return

        if self.goal_reached:
            return

        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        error = self.normalize_angle(self.target_angle - self.current_yaw)

        # 2. Check if goal is reached
        if abs(error) < self.angle_tolerance:
            if not self.goal_reached:
                self.get_logger().info('TARGET REACHED!')
                self.get_logger().info(f'Final heading: {math.degrees(self.current_yaw):.2f}°')
                self.goal_reached = True

                # Stop the robot
                msg = Twist()
                self.publisher.publish(msg)
            return

        p_term = self.kp * error
        self.integral += error * dt
        max_integral = 0.5
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        angular_velocity = p_term + i_term + d_term

        angular_velocity = max(-self.max_angular_speed,
                             min(self.max_angular_speed, angular_velocity))


        if abs(angular_velocity) < self.min_angular_speed:
            angular_velocity = math.copysign(self.min_angular_speed, angular_velocity)

        self.previous_error = error

        msg = Twist()
        msg.linear.x = 0.0  
        msg.angular.z = angular_velocity

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Current: {math.degrees(self.current_yaw):6.2f}° | '
            f'Target: {math.degrees(self.target_angle):6.2f}° | '
            f'Error: {math.degrees(error):6.2f}° | '
            f'P: {p_term:.2f} | I: {i_term:.2f} | D: {d_term:.2f} | '
            f'Angular: {angular_velocity:.2f} rad/s',
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    turn_pid_node = TurnPID()

    try:
        rclpy.spin(turn_pid_node)
    except KeyboardInterrupt:
        pass

    # Cleanup - stop the robot
    stop_msg = Twist()
    turn_pid_node.publisher.publish(stop_msg)
    turn_pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
