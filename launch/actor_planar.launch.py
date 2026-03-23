import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_lpl = get_package_share_directory('lpl_demo')

    # Force Waffle Pi (For Camera)
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

    # Point to our NEW World File
    world_file = os.path.join(pkg_lpl, 'worlds', 'actor_puppet.world')

    # Start Gazebo w
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn Only the Robot 
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pos': '-5.0', 'y_pos': '0.0'}.items()
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot
    ])