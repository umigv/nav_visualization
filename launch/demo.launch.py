from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('nav_visualization')
    
    # Path to costmap file
    costmap_path = os.path.join(pkg_dir, 'config', 'costmap.txt')
    
    return LaunchDescription([
        # Visualization Node
        Node(
            package='nav_visualization',
            executable='visualizer_node',
            name='costmap_visualizer',
            output='screen',
            parameters=[{
                'window_height': 600,
                'window_width': 600,
                'grid_height': 20,
                'grid_width': 20,
                'costmap_file': costmap_path
            }]
        ),
        
        # Dummy Publisher Node
        Node(
            package='nav_visualization',
            executable='dummy_publisher',
            name='dummy_publisher',
            output='screen',
            parameters=[{
                'grid_width': 20,
                'grid_height': 20,
                'update_rate': 1.0
            }]
        )
    ])