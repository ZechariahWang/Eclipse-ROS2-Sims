#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.min_lookahead_distance = 0.2
        self.base_lookahead_distance = 0.3
        self.max_lookahead_distance = 0.5
        self.max_linear_speed = 1.5  # Reduced for better control
        self.min_linear_speed = 0.2
        self.max_angular_speed = 2.0
        self.waypoint_tolerance = 0.15  # Tighter tolerance
        self.slow_down_distance = 0.6  # Start slowing earlier
        self.crawl_distance = 0.3  # Very slow when this close 

        self.path = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0)
        ]

        self.current_waypoint_index = 0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.start_x = None
        self.start_y = None
        self.goal_reached = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Pure Pursuit Controller Started')
        self.get_logger().info(f'Base lookahead distance: {self.base_lookahead_distance}m')
        self.get_logger().info(f'Path waypoints: {len(self.path)}')
        self.get_logger().info(f'Max linear speed: {self.max_linear_speed} m/s')

    def normalize_angle(self, angle):
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

    def calculate_curvature(self, lookahead_point):
        lx, ly = lookahead_point

        dx = lx - self.current_x
        dy = ly - self.current_y
        local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)

        ld = math.sqrt(local_x**2 + local_y**2)

        if ld < 0.01:  
            return 0.0

        curvature = (2.0 * local_y) / (ld ** 2)
        return curvature

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

                msg = Twist()
                self.publisher.publish(msg)
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
                msg = Twist()
                self.publisher.publish(msg)
                return

            # Update target for new waypoint
            target_x, target_y = self.path[self.current_waypoint_index]
            distance_to_waypoint = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        # Adaptive lookahead distance based on distance to waypoint
        if distance_to_waypoint < self.crawl_distance:
            lookahead_distance = self.min_lookahead_distance
        elif distance_to_waypoint < self.slow_down_distance:
            # Smoothly interpolate lookahead
            ratio = (distance_to_waypoint - self.crawl_distance) / (self.slow_down_distance - self.crawl_distance)
            lookahead_distance = self.min_lookahead_distance + ratio * (self.base_lookahead_distance - self.min_lookahead_distance)
        else:
            lookahead_distance = self.base_lookahead_distance

        # Find lookahead point (don't advance waypoint here)
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

        # Use Pure Pursuit curvature formula for angular velocity
        curvature = self.calculate_curvature(lookahead_point)

        # Calculate linear velocity based on distance to waypoint
        if distance_to_waypoint < self.crawl_distance:
            # Very close - crawl speed
            linear_velocity = self.min_linear_speed
        elif distance_to_waypoint < self.slow_down_distance:
            # Approaching - smoothly reduce speed
            speed_ratio = (distance_to_waypoint - self.crawl_distance) / (self.slow_down_distance - self.crawl_distance)
            linear_velocity = self.min_linear_speed + speed_ratio * (self.max_linear_speed - self.min_linear_speed)
        else:
            # Far away - full speed
            linear_velocity = self.max_linear_speed

        # Reduce speed on sharp curves
        if abs(curvature) > 2.0:
            linear_velocity = min(linear_velocity, self.min_linear_speed * 1.5)
        elif abs(curvature) > 1.0:
            linear_velocity = min(linear_velocity, self.max_linear_speed * 0.6)

        # Calculate angular velocity from curvature: omega = v * curvature
        angular_velocity = linear_velocity * curvature
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))

        # Apply additional damping when very close to waypoint
        if distance_to_waypoint < self.waypoint_tolerance * 1.5:
            damping = distance_to_waypoint / (self.waypoint_tolerance * 1.5)
            angular_velocity *= damping
            linear_velocity *= damping

        if not math.isfinite(angular_velocity):
            angular_velocity = 0.0
        if not math.isfinite(linear_velocity):
            linear_velocity = 0.0

        msg = Twist()
        msg.linear.x = float(linear_velocity)
        msg.angular.z = float(angular_velocity)
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'WP: {self.current_waypoint_index+1}/{len(self.path)} '
            f'({target_x:.2f}, {target_y:.2f}) | '
            f'Dist: {distance_to_waypoint:.3f}m | '
            f'LA: {lookahead_distance:.2f}m | '
            f'Curv: {curvature:.2f} | '
            f'V: {linear_velocity:.2f} m/s | W: {angular_velocity:.2f} rad/s',
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    pp_node = PurePursuit()

    try:
        rclpy.spin(pp_node)
    except KeyboardInterrupt:
        pass

    stop_msg = Twist()
    pp_node.publisher.publish(stop_msg)
    pp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

