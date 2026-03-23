import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_lpl_demo = get_package_share_directory('lpl_demo')

    # Path to our new world file
    world_path = os.path.join(pkg_lpl_demo, 'worlds', 'gauntlet2.world')

    return LaunchDescription([
        # 1. Start Gazebo with the Gauntlet World
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        # 2. Spawn the Robot at 0,0
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'turtlebot3_waffle_pi',
                 '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
                 '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'
        ),
    ])