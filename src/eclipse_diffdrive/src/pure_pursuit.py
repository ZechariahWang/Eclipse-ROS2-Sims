#!/usr/bin/env python3
# wip not yet added
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion

class PurePursuit(Node):
    def __init__(self):
        super().__init__('move_to_point')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.kp = 2.0
        self.ki = 0
        self.kd = 0.5

        self.angular_kp = 3
        self.angular_ki = 0
        self.angular_kd = 0.5

        self.target_x = 2.0
        self.target_y = 2.0

        self.max_linear_speed = 2
        self.max_angular_speed = 2
        self.distance_tolerance = 1

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.previous_error = 0.0
        self.integral = 0.0
        self.angular_previous_error = 0.0
        self.angular_integral = 0.0
        self.last_time = self.get_clock().now()

        self.goal_reached = False
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Move To Point Controller Started')
        self.get_logger().info(f'Target position: ({self.target_x}, {self.target_y})')
        self.get_logger().info(f'Linear PID - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Angular PID - Kp: {self.angular_kp}, Ki: {self.angular_ki}, Kd: {self.angular_kd}')

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

        # Extract current yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')
            self.get_logger().info(f'Starting heading: {math.degrees(self.current_yaw):.2f}°')

    def control_loop(self):
        if self.current_x is None or self.current_yaw is None:
            self.get_logger().warn('No odometry data available', throttle_duration_sec=2.0)
            return

        if self.goal_reached:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        # Calculate desired heading to target
        desired_heading = math.atan2(dy, dx)
        # Calculate angular error relative to current heading
        angular_error = self.normalize_angle(desired_heading - self.current_yaw)
        distance_error = math.sqrt(dx**2 + dy**2)

        if abs(distance_error) < self.distance_tolerance:
            if not self.goal_reached:
                self.get_logger().info('TARGET REACHED!')
                self.get_logger().info(f'Final position: ({self.current_x:.3f}, {self.current_y:.3f})')
                self.goal_reached = True

                msg = Twist()
                self.publisher.publish(msg)
            return

        p = self.kp * distance_error
        self.integral += distance_error * dt
        self.integral = max(-1.0, min(1.0, self.integral))
        i = self.ki * self.integral
        derivative = (distance_error - self.previous_error) / dt
        d = self.kd * derivative

        angular_p = self.angular_kp * angular_error
        self.angular_integral += angular_error * dt
        self.angular_integral = max(-1.0, min(1.0, self.angular_integral))
        angular_i = self.angular_ki * self.angular_integral
        angular_derivative = (angular_error - self.angular_previous_error) / dt
        angular_d = self.angular_kd * angular_derivative

        linear_velocity = p + i + d
        linear_velocity = max(-self.max_linear_speed, min(self.max_linear_speed, linear_velocity))

        angular_velocity = angular_p + angular_i + angular_d
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))

        # Ensure values are valid floats
        if not math.isfinite(angular_velocity):
            angular_velocity = 0.0
        if not math.isfinite(linear_velocity):
            linear_velocity = 0.0

        self.previous_error = distance_error
        self.angular_previous_error = angular_error

        msg = Twist()
        msg.angular.z = float(angular_velocity)

        # Only move forward if roughly aligned with target (within 15 degrees)
        if abs(angular_error) < math.radians(15):
            msg.linear.x = float(linear_velocity)
        else:
            msg.linear.x = 0.0  # Stop and turn first

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
            f'Heading: {math.degrees(self.current_yaw):.1f}° | '
            f'Target: ({self.target_x:.2f}, {self.target_y:.2f}) | '
            f'Dist: {distance_error:.3f}m | '
            f'Ang Error: {math.degrees(angular_error):.1f}° | '
            f'Lin: {msg.linear.x:.2f} m/s | Ang: {angular_velocity:.2f} rad/s',
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    mtp_node = PurePursuit()

    try:
        rclpy.spin(mtp_node)
    except KeyboardInterrupt:
        pass

    stop_msg = Twist()
    mtp_node.publisher.publish(stop_msg)
    mtp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

