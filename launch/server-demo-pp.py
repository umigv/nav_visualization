from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys



def generate_launch_description():
    # Get the costmap file name from the command line
    # if len(sys.argv) < 5:
    #     print("Usage: ros2 launch nav_visualization server-demo-pp.py <costmap_file>")
    #     sys.exit(1)
    
    costmap_file_launch_arg = DeclareLaunchArgument(
        'costmap_file',
        default_value='costmap.txt',
        description='Path to the costmap file'
    )
    costmap_file = LaunchConfiguration('costmap_file')


    return LaunchDescription([
        costmap_file_launch_arg,
        # Visualization Node
        Node( 
            package='nav_visualization',
            executable='path_planning_visualizer',
            name='path_planning_visualizer',
            output='screen',
            parameters=[{
                'costmap_file': costmap_file
            }]
        ),
        Node(
            package='planner_server',
            executable ='planner_server',
            name='path_planning_server',
            parameters=[{
              'planner_plugin': "ExamplePathPlannerPlugin",
              'odom_topic': "/odom"
            }],
            emulate_tty=True    # Enable colors and formatting
        )
    ])