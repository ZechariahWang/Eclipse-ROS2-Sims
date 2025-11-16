#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


class PurePursuitAckermann(Node):
    def __init__(self):
        super().__init__('pure_pursuit_ackermann')

        self.rear_left_pub = self.create_publisher(Float64, 'rear_left_wheel/cmd_vel', 10)
        self.rear_right_pub = self.create_publisher(Float64, 'rear_right_wheel/cmd_vel', 10)
        self.front_left_steer_pub = self.create_publisher(Float64, 'front_left_steering/cmd_pos', 10)
        self.front_right_steer_pub = self.create_publisher(Float64, 'front_right_steering/cmd_pos', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.min_lookahead_distance = 0.4
        self.base_lookahead_distance = 0.6
        self.max_lookahead_distance = 1.0
        self.max_speed = 2.0  # m/s
        self.min_speed = 0.3
        self.max_steering_angle = 0.6  # radians (~34 degrees)
        self.waypoint_tolerance = 0.2
        self.slow_down_distance = 0.8
        self.crawl_distance = 0.4

        # Vehicle parameters (must match URDF)
        self.wheelbase = 0.3  # Distance between front and rear axles
        self.track_width = 0.25  # Distance between left and right wheels
        self.wheel_radius = 0.05  # Wheel radius in meters

        # Path waypoints (x, y)
        self.path = [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (4.0, 2.0),
            (4.0, 4.0)
        ]

        self.current_waypoint_index = 0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.start_x = None
        self.start_y = None
        self.goal_reached = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Pure Pursuit Ackermann Controller Started')
        self.get_logger().info(f'Base lookahead distance: {self.base_lookahead_distance}m')
        self.get_logger().info(f'Path waypoints: {len(self.path)}')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m')

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')
            self.get_logger().info(f'Starting heading: {math.degrees(self.current_yaw):.2f}°')

    def calculate_ackermann_steering(self, lookahead_point):
        """Calculate left and right steering angles using Ackermann geometry"""
        lx, ly = lookahead_point

        # Transform lookahead point to robot frame
        dx = lx - self.current_x
        dy = ly - self.current_y

        # Rotate to robot's local frame
        local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)

        # Calculate lookahead distance
        ld = math.sqrt(local_x**2 + local_y**2)

        if ld < 0.01:
            return 0.0, 0.0

        # Pure Pursuit formula for center steering angle
        # steering_angle = atan(2 * L * sin(alpha) / ld)
        alpha = math.atan2(local_y, local_x)
        center_steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)

        # Convert center steering to left and right using Ackermann geometry
        if abs(center_steer) < 0.001:
            # Going straight
            left_steer = 0.0
            right_steer = 0.0
        else:
            # Calculate turn radius from center steering angle
            turn_radius = self.wheelbase / math.tan(center_steer)

            # Calculate left and right steering angles
            left_steer = math.atan(self.wheelbase / (turn_radius - self.track_width / 2))
            right_steer = math.atan(self.wheelbase / (turn_radius + self.track_width / 2))

        return left_steer, right_steer

    def control_loop(self):
        if self.current_x is None or self.current_yaw is None:
            self.get_logger().warn('No odometry data available', throttle_duration_sec=2.0)
            return

        if self.goal_reached:
            return

        if self.current_waypoint_index >= len(self.path):
            if not self.goal_reached:
                self.get_logger().info('PATH COMPLETED!')
                self.get_logger().info(f'Final position: ({self.current_x:.3f}, {self.current_y:.3f})')
                self.goal_reached = True

                # Stop all joints
                stop_msg = Float64()
                stop_msg.data = 0.0
                self.front_left_steer_pub.publish(stop_msg)
                self.front_right_steer_pub.publish(stop_msg)
                self.rear_left_pub.publish(stop_msg)
                self.rear_right_pub.publish(stop_msg)
            return

        # Calculate distance to current waypoint
        target_x, target_y = self.path[self.current_waypoint_index]
        distance_to_waypoint = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        # Check if we should advance to next waypoint
        if distance_to_waypoint < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}/{len(self.path)}')

            if self.current_waypoint_index >= len(self.path):
                self.get_logger().info('PATH COMPLETED!')
                self.get_logger().info(f'Final position: ({self.current_x:.3f}, {self.current_y:.3f})')
                self.goal_reached = True

                # Stop all joints
                stop_msg = Float64()
                stop_msg.data = 0.0
                self.front_left_steer_pub.publish(stop_msg)
                self.front_right_steer_pub.publish(stop_msg)
                self.rear_left_pub.publish(stop_msg)
                self.rear_right_pub.publish(stop_msg)
                return

            # Update target for new waypoint
            target_x, target_y = self.path[self.current_waypoint_index]
            distance_to_waypoint = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        # Adaptive lookahead distance
        if distance_to_waypoint < self.crawl_distance:
            lookahead_distance = self.min_lookahead_distance
        elif distance_to_waypoint < self.slow_down_distance:
            ratio = (distance_to_waypoint - self.crawl_distance) / (self.slow_down_distance - self.crawl_distance)
            lookahead_distance = self.min_lookahead_distance + ratio * (self.base_lookahead_distance - self.min_lookahead_distance)
        else:
            lookahead_distance = self.base_lookahead_distance

        # Find lookahead point
        lookahead_point = None
        for i in range(self.current_waypoint_index, len(self.path)):
            wx, wy = self.path[i]
            dist = math.sqrt((wx - self.current_x)**2 + (wy - self.current_y)**2)
            if dist >= lookahead_distance:
                lookahead_point = (wx, wy)
                break

        # If no point at lookahead distance, use current target
        if lookahead_point is None:
            lookahead_point = (target_x, target_y)

        # Calculate steering angles using Pure Pursuit
        left_steer, right_steer = self.calculate_ackermann_steering(lookahead_point)
        left_steer = max(-self.max_steering_angle, min(self.max_steering_angle, left_steer))
        right_steer = max(-self.max_steering_angle, min(self.max_steering_angle, right_steer))

        # Calculate average steering for speed control
        avg_steer = (left_steer + right_steer) / 2.0

        # Calculate speed based on distance to waypoint and steering angle
        if distance_to_waypoint < self.crawl_distance:
            speed = self.min_speed
        elif distance_to_waypoint < self.slow_down_distance:
            speed_ratio = (distance_to_waypoint - self.crawl_distance) / (self.slow_down_distance - self.crawl_distance)
            speed = self.min_speed + speed_ratio * (self.max_speed - self.min_speed)
        else:
            speed = self.max_speed

        # Reduce speed on sharp turns
        if abs(avg_steer) > math.radians(20):
            speed = min(speed, self.min_speed * 2.0)
        elif abs(avg_steer) > math.radians(10):
            speed = min(speed, self.max_speed * 0.6)

        # Apply damping when very close to waypoint
        if distance_to_waypoint < self.waypoint_tolerance * 1.5:
            damping = distance_to_waypoint / (self.waypoint_tolerance * 1.5)
            speed *= damping
            left_steer *= damping
            right_steer *= damping

        if not math.isfinite(left_steer):
            left_steer = 0.0
        if not math.isfinite(right_steer):
            right_steer = 0.0
        if not math.isfinite(speed):
            speed = 0.0

        # Convert linear speed to wheel angular velocity
        wheel_velocity = speed / self.wheel_radius

        # Publish commands to individual joints
        left_steer_msg = Float64()
        left_steer_msg.data = float(left_steer)
        self.front_left_steer_pub.publish(left_steer_msg)

        right_steer_msg = Float64()
        right_steer_msg.data = float(right_steer)
        self.front_right_steer_pub.publish(right_steer_msg)

        left_vel_msg = Float64()
        left_vel_msg.data = float(wheel_velocity)
        self.rear_left_pub.publish(left_vel_msg)

        right_vel_msg = Float64()
        right_vel_msg.data = float(wheel_velocity)
        self.rear_right_pub.publish(right_vel_msg)

        self.get_logger().info(
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'WP: {self.current_waypoint_index+1}/{len(self.path)} '
            f'({target_x:.2f}, {target_y:.2f}) | '
            f'Dist: {distance_to_waypoint:.3f}m | '
            f'LA: {lookahead_distance:.2f}m | '
            f'Speed: {speed:.2f} m/s | L/R Steer: {math.degrees(left_steer):.1f}/{math.degrees(right_steer):.1f}°',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    pp_node = PurePursuitAckermann()

    try:
        rclpy.spin(pp_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the vehicle
        if rclpy.ok():
            try:
                stop_msg = Float64()
                stop_msg.data = 0.0
                pp_node.front_left_steer_pub.publish(stop_msg)
                pp_node.front_right_steer_pub.publish(stop_msg)
                pp_node.rear_left_pub.publish(stop_msg)
                pp_node.rear_right_pub.publish(stop_msg)
            except:
                pass

            pp_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
