import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Path to our NEW dynamic world file
    pkg_lpl_demo = get_package_share_directory('lpl_demo')
    world_file = os.path.join(pkg_lpl_demo, 'worlds', 'dynamic_tests.world')

    # 2. Path to standard Gazebo and TurtleBot launch files
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_gazebo')

    # 3. Start Gazebo Server with our World
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 4. Start Gazebo Client (The window you see)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 5. Spawn the Robot at (0,0) facing East
    robot_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pos': '0.0', 
            'y_pos': '0.0',
            'z_pos': '0.01'
        }.items()
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_spawn_cmd
    ])