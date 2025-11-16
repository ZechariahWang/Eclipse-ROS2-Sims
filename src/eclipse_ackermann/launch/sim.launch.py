from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='eclipse_ackermann').find('eclipse_ackermann')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'eclipse_ackermann.urdf.xacro')

    # Convert Xacro → URDF → SDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'eclipse_ackermann.urdf')
    sdf_file = os.path.join(pkg_share, 'urdf', 'eclipse_ackermann.sdf')
    os.system(f"xacro {xacro_path} -o {urdf_file}")
    os.system(f"ign sdf -p {urdf_file} > {sdf_file} 2>/dev/null")

    # PROTOBUF request for spawning using file path
    robot_req = f'name: "eclipse_ackermann_bot" sdf_filename: "{sdf_file}" pose: {{ position: {{ x: 0.0 y: 0.0 z: 0.10 }} }}'

    gz_spawn_entity = [
        'ign', 'service', '-s', '/world/default/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '5000',
        '--req', robot_req
    ]

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
            output='screen'
        ),

        # ROS-Gazebo bridge for joint control and sensors
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/rear_left_wheel/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
                '/rear_right_wheel/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
                '/front_left_steering/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/front_right_steering/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/world/default/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'
            ],
            output='screen'
        ),

        # Pose to Odometry converter
        Node(
            package='eclipse_ackermann',
            executable='pose_to_odom.py',
            output='screen'
        ),

        # Wait a few seconds for Gazebo to load, then spawn robot
        TimerAction(
            period=5.0,
            actions=[ExecuteProcess(cmd=gz_spawn_entity, output='screen')]
        ),
    ])
