import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class DistancePID(Node):
    def __init__(self):
        super().__init__('distance_pid')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.5

        self.target_distance = 2.0  
        self.max_speed = 0.5            
        self.distance_tolerance = 0.05  

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.distance_traveled = 0.0

        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        self.goal_reached = False
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Distance PID Controller Started')
        self.get_logger().info(f'Target distance: {self.target_distance}m')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        self.distance_traveled = math.sqrt(dx**2 + dy**2)

    def control_loop(self):
        if self.current_x is None:
            self.get_logger().warn('No odometry data available', throttle_duration_sec=2.0)
            return

        if self.goal_reached:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return
        
        error = self.target_distance - self.distance_traveled
        if abs(error) < self.distance_tolerance:
            if not self.goal_reached:
                self.get_logger().info('TARGET DISTANCE REACHED!')
                self.get_logger().info(f'Traveled: {self.distance_traveled:.3f}m')
                self.goal_reached = True

                msg = Twist()
                self.publisher.publish(msg)
            return

        p = self.kp * error
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))
        i = self.ki * self.integral
        derivative = (error - self.previous_error) / dt
        d = self.kd * derivative

        velocity = p+i+d
        velocity = max(-self.max_speed, min(self.max_speed, velocity))
        self.previous_error = error

        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = 0.0 

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Traveled: {self.distance_traveled:.3f}m | '
            f'Target: {self.target_distance:.3f}m | '
            f'Error: {error:.3f}m | '
            f'P: {p:.2f} | I: {i:.2f} | D: {d:.2f} | '
            f'Velocity: {velocity:.2f} m/s',
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    distance_pid_node = DistancePID()

    try:
        rclpy.spin(distance_pid_node)
    except KeyboardInterrupt:
        pass

    stop_msg = Twist()
    distance_pid_node.publisher.publish(stop_msg)
    distance_pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

