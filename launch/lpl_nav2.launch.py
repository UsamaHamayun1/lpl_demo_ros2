import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    tb3_nav_dir = get_package_share_directory('turtlebot3_navigation2')
    
    return LaunchDescription([
        # 1. Allow you to pass paths from the terminal
        DeclareLaunchArgument('map', default_value='map.yaml'),
        DeclareLaunchArgument('params_file', default_value='nav2_params.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 2. THE MAGIC LINE: Intercept Nav2's motor commands
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        
        # 3. Launch standard Nav2, passing your files down
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_nav_dir, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map': LaunchConfiguration('map'), 
                'params_file': LaunchConfiguration('params_file')
            }.items()
        )
    ])