from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to TurtleBot3 Gazebo package
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_launch_file = os.path.join(
        turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py'
    )

    # Path to your RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'config',
        'rviz.rviz'
    )

    return LaunchDescription([
        # Launch TurtleBot3 Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_launch_file)
        ),

        # Launch your custom DWA planner node
        Node(
            package='custom_dwa_planner',
            executable='plannar',  # must match entry_points in setup.py
            name='dwa_local_planner',
            output='screen'
        ),

        # Launch RViz2 with your config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
