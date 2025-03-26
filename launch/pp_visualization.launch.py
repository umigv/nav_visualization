#!/usr/bin/env python3

"""
Launch File for Path Planning Visualizer Node

This launch file starts two ROS2 nodes from the 'nav_visualization' package:
1. **path_planning_visualizer**: Visualizes path planning using a provided costmap.

Launch Arguments:
    - costmap_file: Path to the costmap file (default: 'costmap.txt').
    - window_height: Height of the visualization window in pixels (default: '800').
    - window_width: Width of the visualization window in pixels (default: '800').

Usage:
    ros2 launch <package_name> <launch_file_name>.py costmap_file:=my_map.txt window_height:=600 window_width:=800

Dependencies:
    - ROS2 (launch, launch_ros)
    - Python 3.x
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = [
        PathJoinSubstitution([
        FindPackageShare('nav_visualization'),
        'config',
        'pp_vis_params.yaml'
    ])]
    # # Declare launch arguments for dynamic configuration
    # costmap_file_arg = DeclareLaunchArgument(
    #     'costmap_file',
    #     default_value='costmap.txt',
    #     description='Path to the costmap file'
    # )

    # window_height_arg = DeclareLaunchArgument(
    #     'window_height',
    #     default_value='800',  # Provide a default value to avoid errors
    #     description='Height of the window in pixels'
    # )

    # window_width_arg = DeclareLaunchArgument(
    #     'window_width',
    #     default_value='800',  # Provide a default value to avoid errors
    #     description='Width of the window in pixels'
    # )

    # # Create LaunchConfiguration variables from the declared arguments
    # costmap_file = LaunchConfiguration('costmap_file')
    # window_height = LaunchConfiguration('window_height')
    # window_width = LaunchConfiguration('window_width')

    return LaunchDescription([

        # Launch the Path Planning Visualizer Node
        Node(
            package='nav_visualization',
            executable='path_planning_visualizer',
            name='path_planning_visualizer',
            output='screen',
            parameters=params
        )
    ])