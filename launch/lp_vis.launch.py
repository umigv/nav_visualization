#!/usr/bin/env python3

"""
ROS2 Launch File for Local Planning Visualization Node

This launch file starts two ROS2 nodes:
1. **local_planning_visualizer**: A visualization node that displays a costmap and robot navigation.

Usage:
    ros2 launch <package_name> <launch_file_name>.py costmap_file:=my_map.txt window_height:=800 window_width:=600 topic:=/cmd_vel

Launch Arguments:
    - costmap_file: Path to the costmap file (default: 'costmap.txt')
    - window_height: Height of the visualization window in pixels (default: '800')
    - window_width: Width of the visualization window in pixels (default: '800')
    - topic: ROS2 topic to publish Twist messages (default: '/robot_twist')

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
        'lp_vis_params.yaml'
    ])]
    # # Declare launch arguments for dynamic configuration
    # costmap_file_arg = DeclareLaunchArgument(
    #     'costmap_file',
    #     default_value='costmap.txt',
    #     description='Path to the costmap file.'
    # )

    # window_height_arg = DeclareLaunchArgument(
    #     'window_height',
    #     default_value='800',
    #     description='Height of the visualization window in pixels.'
    # )

    # window_width_arg = DeclareLaunchArgument(
    #     'window_width',
    #     default_value='800',
    #     description='Width of the visualization window in pixels.'
    # )

    # topic_arg = DeclareLaunchArgument(
    #     'topic',
    #     default_value='/cmd_vel',
    #     description='ROS2 topic to publish Twist messages.'
    # )

    # Use LaunchConfiguration to substitute the declared arguments
    # costmap_file = LaunchConfiguration('costmap_file')
    # window_height = LaunchConfiguration('window_height')
    # window_width = LaunchConfiguration('window_width')
    # topic = LaunchConfiguration('topic')

    # Create and return the LaunchDescription object with both nodes
    return LaunchDescription([
        # # Declare the launch arguments
        # costmap_file_arg,
        # window_height_arg,
        # window_width_arg,
        # topic_arg,

        # Visualization Node: local_planning_visualizer
        Node(
            package='nav_visualization',
            executable='lp_vis',
            name='local_planning_visualizer',
            output='screen',
            # parameters=[{
            #     'costmap_file': costmap_file,
            #     'window_height': window_height,
            #     'window_width': window_width,
            #     'topic': topic
            # }]
            parameters=params
        ),
    ])