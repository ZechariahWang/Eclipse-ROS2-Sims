from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='eclipse_diffdrive').find('eclipse_diffdrive')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'eclipse_diffdrive.urdf.xacro')

    # Convert Xacro → URDF → SDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'eclipse_diffdrive.urdf')
    sdf_file = os.path.join(pkg_share, 'urdf', 'eclipse_diffdrive.sdf')
    os.system(f"xacro {xacro_path} -o {urdf_file}")
    os.system(f"ign sdf -p {urdf_file} > {sdf_file} 2>/dev/null")

    # PROTOBUF request for spawning using file path
    robot_req = f'name: "eclipse_bot" sdf_filename: "{sdf_file}" pose: {{ position: {{ x: 0.0 y: 0.0 z: 0.10 }} }}'

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

        # ROS-Gazebo bridge for cmd_vel topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
            ],
            output='screen'
        ),

        # Wait a few seconds for Gazebo to load, then spawn robot
        TimerAction(
            period=5.0,
            actions=[ExecuteProcess(cmd=gz_spawn_entity, output='screen')]
        ),
    ])
