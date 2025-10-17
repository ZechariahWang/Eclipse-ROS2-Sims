from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='eclipse_diffdrive').find('eclipse_diffdrive')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world.sdf')
    robot_path = os.path.join(pkg_share, 'urdf', 'eclipse_diffdrive.urdf.xacro')

    # Command to spawn the robot in Gazebo
    gz_spawn_entity = [
        'gz', 'service', '-s', '/world/default/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',  # 5 seconds timeout
        '--req',
        f'{{"name": "eclipse_bot", "sdf_filename": "{robot_path}", '
        '"pose": {"position": {"x": 0, "y": 0, "z": 0.1}}}}'
    ]

    return LaunchDescription([
        # Start Gazebo with your world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        # Wait 5 seconds for world to load, then spawn robot
        TimerAction(
            period=5.0,
            actions=[ExecuteProcess(cmd=gz_spawn_entity, output='screen')]
        ),
    ])

