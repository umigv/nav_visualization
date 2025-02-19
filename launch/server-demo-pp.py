from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Get the package share directory
    #     

    pkg_dir = get_package_share_directory('nav_visualization')
    costmap_path = os.path.join(pkg_dir, 'costmap4.txt')

    return LaunchDescription([
        # Visualization Node
        Node(
            package='nav_visualization',
            executable='path_planning_visualizer',
            name='path_planning_visualizer',
            output='screen',
            parameters=[{
              'costmap_file': costmap_path,
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