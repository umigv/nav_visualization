#!/usr/bin/env python3

"""
ROS2 Launch File for Path Planning Visualizer and Planner Server

This launch file starts two ROS2 nodes:
1. **path_planning_visualizer**: A visualization node from the 'nav_visualization' package
   that displays a costmap using the provided parameters.
2. **planner_server**: A planning server node from the 'planner_server' package that
   uses a specified planner plugin and listens to an odometry topic.

Launch Arguments:
    - costmap_file: Path to the costmap file (default: 'costmap.txt').
    - window_height: Height of the visualization window in pixels (default: '800').
    - window_width: Width of the visualization window in pixels (default: '800').

Usage:
    ros2 launch <package_name> <launch_file_name>.py \
        costmap_file:=my_map.txt window_height:=600 window_width:=800

Dependencies:
    - ROS2 (launch, launch_ros)
    - Python 3.x
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the 'costmap_file' launch argument with a default value.
    costmap_file_arg = DeclareLaunchArgument(
        'costmap_file',
        default_value='costmap.txt',
        description='Path to the costmap file'
    )
    costmap_file = LaunchConfiguration('costmap_file')
    
    # Declare the 'window_height' launch argument with a default value.
    window_height_arg = DeclareLaunchArgument(
        'window_height',
        default_value='800',  # Default height in pixels
        description='Height of the window in pixels'
    )
    window_height = LaunchConfiguration('window_height')

    # Declare the 'window_width' launch argument with a default value.
    window_width_arg = DeclareLaunchArgument(
        'window_width',
        default_value='800',  # Default width in pixels
        description='Width of the window in pixels'
    )
    window_width = LaunchConfiguration('window_width')

    # Create the LaunchDescription with all declared launch arguments and nodes.
    return LaunchDescription([
        # Declare launch arguments.
        costmap_file_arg,
        window_height_arg,
        window_width_arg,

        # Visualization Node: displays the costmap and visualizes the planning.
        Node(
            package='nav_visualization',
            executable='path_planning_visualizer',
            name='path_planning_visualizer',
            output='screen',
            parameters=[{
                'costmap_file': costmap_file,
                'window_height': window_height,
                'window_width': window_width
            }]
        ),

        # Planner Server Node: runs the planner server with a specified plugin and odom topic.
        Node(
            package='planner_server',
            executable='planner_server',
            name='path_planning_server',
            output='screen',
            parameters=[{
                'planner_plugin': "ExamplePathPlannerPlugin",
                'odom_topic': "/odom"
            }],
            emulate_tty=True  # Enables colors and formatting in the console output.
        )
    ])