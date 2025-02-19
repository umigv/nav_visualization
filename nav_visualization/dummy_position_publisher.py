#!/usr/bin/env python3

"""
Dummy Position Publisher for ROS2

This script publishes random robot positions on a specified ROS2 topic at a configurable rate.
It reads a costmap from a file, determines the grid dimensions, and ensures the robot's 
movements stay within bounds.

Features:
- Loads a costmap from a specified file.
- Publishes random positions within the costmap at a specified update rate.
- Ensures movements stay within the grid boundaries.
- Publishes messages to a specified ROS2 topic.

Usage:
1. Run a ROS2 environment.
2. Execute this script.
3. The node will start publishing random positions.
4. Use `ros2 topic echo /robot_position` to see the published positions.
5. Stop the script with Ctrl+C.

Dependencies:
- ROS2 (rclpy)
- numpy
- os
- random

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random
import os
import numpy as np


class DummyPositionPublisher(Node):
    """
    A ROS2 node that publishes random robot positions within a given costmap.

    The node:
    - Reads a costmap to determine the grid size.
    - Publishes random positions within the grid at a configurable update rate.
    - Ensures movements remain within the bounds of the costmap.
    """

    def __init__(self):
        super().__init__('position_publisher')

        # Declare and retrieve parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('costmap_file', 'costmap.txt'),
                ('update_rate', 1.0),
                ('topic', '/robot_position')
            ]
        )

        self.costmap_file = self.get_parameter('costmap_file').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        # Resolve costmap file path
        script_directory = os.path.dirname(os.path.abspath(__file__))
        for _ in range(6):  # Move up 6 directories
            script_directory = os.path.dirname(script_directory)

        costmap_path = os.path.join(script_directory, "src", "nav_visualization", "costmaps", self.costmap_file)

        # Load costmap
        self.costmap = self.read_costmap(costmap_path)
        self.grid_height, self.grid_width = self.costmap.shape

        # Initialize the publisher
        self.publisher = self.create_publisher(Point, self.topic, 10)

        # Set initial position at the center of the grid
        self.current_x = self.grid_width // 2
        self.current_y = self.grid_height // 2

        # Create a timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_position)

    def publish_position(self):
        """
        Generates and publishes a new random position while ensuring it remains within grid boundaries.
        """
        # Randomly move the position within a small range
        new_x = self.current_x + random.randint(-2, 2)
        new_y = self.current_y + random.randint(-2, 2)

        # Ensure position stays within grid bounds
        new_x = max(0, min(self.grid_width - 1, new_x))
        new_y = max(0, min(self.grid_height - 1, new_y))

        # Update position
        self.current_x = new_x
        self.current_y = new_y

        # Create and publish the position message
        msg = Point()
        msg.x = float(new_x)
        msg.y = float(new_y)
        msg.z = 0.0  # Z-axis is unused but included for compatibility

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing position: ({new_x}, {new_y})')

    def read_costmap(self, file_path):
        """
        Reads the costmap from a file and extracts the start and goal positions.

        Args:
            file_path (str): Path to the costmap file.

        Returns:
            np.ndarray: The costmap as a NumPy array.
        """
        with open(file_path, 'r') as f:
            lines = f.readlines()
            self.start_position = list(map(int, lines[0].split()))
            self.goal_position = list(map(int, lines[1].split()))
            return np.array([[int(num) for num in line.split()] for line in lines[2:]])


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    publisher = DummyPositionPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()