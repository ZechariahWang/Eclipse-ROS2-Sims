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

        self.lookahead_distance = 0.5
        self.lookahead_gain = 2.0  
        self.angular_gain = 4.0
        self.max_linear_speed = 3
        self.min_linear_speed = 0.5
        self.max_angular_speed = 3.0
        self.waypoint_tolerance = 0.15  

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
        self.get_logger().info(f'Lookahead distance: {self.lookahead_distance}m')
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

    def find_lookahead_point(self):
        if self.current_waypoint_index >= len(self.path):
            return None

        for i in range(self.current_waypoint_index, len(self.path)):
            wx, wy = self.path[i]
            distance = math.sqrt((wx - self.current_x)**2 + (wy - self.current_y)**2)

            if i == self.current_waypoint_index and distance < self.waypoint_tolerance:
                self.current_waypoint_index += 1
                self.get_logger().info(f'Reached waypoint {i+1}/{len(self.path)}')
                continue

            if distance >= self.lookahead_distance:
                return (wx, wy)

        if self.current_waypoint_index < len(self.path):
            return self.path[-1]

        return None

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

        lookahead_point = self.find_lookahead_point()

        if lookahead_point is None:
            if not self.goal_reached:
                self.get_logger().info('PATH COMPLETED!')
                self.get_logger().info(f'Final position: ({self.current_x:.3f}, {self.current_y:.3f})')
                self.goal_reached = True

                msg = Twist()
                self.publisher.publish(msg)
            return

        lx, ly = lookahead_point
        dx = lx - self.current_x
        dy = ly - self.current_y
        desired_heading = math.atan2(dy, dx)

        heading_error = self.normalize_angle(desired_heading - self.current_yaw)
        angular_velocity = self.angular_gain * heading_error
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))

        if abs(heading_error) > math.radians(45):
            linear_velocity = self.min_linear_speed
        elif abs(heading_error) > math.radians(20):
            linear_velocity = self.max_linear_speed * 0.6
        else:
            linear_velocity = self.max_linear_speed

        if not math.isfinite(angular_velocity):
            angular_velocity = 0.0
        if not math.isfinite(linear_velocity):
            linear_velocity = 0.0

        msg = Twist()
        msg.linear.x = float(linear_velocity)
        msg.angular.z = float(angular_velocity)
        self.publisher.publish(msg)

        target_x, target_y = self.path[self.current_waypoint_index]
        distance_to_target = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        self.get_logger().info(
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'Waypoint: {self.current_waypoint_index+1}/{len(self.path)} | '
            f'Target: ({target_x:.2f}, {target_y:.2f}) | '
            f'Dist: {distance_to_target:.3f}m | '
            f'Lookahead: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f}) | '
            f'Heading Err: {math.degrees(heading_error):.1f}° | '
            f'Lin: {linear_velocity:.2f} m/s | Ang: {angular_velocity:.2f} rad/s',
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

