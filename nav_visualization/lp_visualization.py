#!/usr/bin/env python3

"""
Local Planning Visualizer for ROS2

This script visualizes a robot's movement on a costmap using pygame. It listens to a 
ROS2 Twist topic and updates the robot's position accordingly. The costmap is loaded 
from a text file, and the robot's path is drawn in real-time.

Features:
- Loads a costmap from a specified file.
- Subscribes to a Twist message topic to track robot motion.
- Displays the costmap with start/goal positions.
- Draws the robot's trajectory and indicates its direction and velocity.

Usage:
1. Run a ROS2 environment.
2. Execute this script.
3. The window will display the costmap with real-time updates from the Twist topic.
4. Close the pygame window to terminate the script.

Dependencies:
- ROS2 (rclpy)
- pygame
- numpy
- threading
- geometry_msgs.msg.Twist
- ament_index_python
- pyautogui
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import numpy as np
import threading
import math
import os
import pyautogui
from ament_index_python.packages import get_package_share_directory

class LocalPlanningVisualizer(Node):
    """
    A ROS2 node that visualizes the robot's local path using a costmap and Twist messages.
    """
    
    DELTA_TIME = 0.01  # Time step for updating the robot's position

    def __init__(self):
        super().__init__('local_planning_visualizer')
        
        # Declare and get parameters
        self.declare_parameter('costmap_file', 'costmap.txt')
        self.declare_parameter('window_height', None)
        self.declare_parameter('window_width', None)
        self.declare_parameter('topic', '/robot_twist')

        costmap_file = self.get_parameter('costmap_file').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Load costmap
        self.costmap = self.read_costmap(costmap_file)
        self.grid_height, self.grid_width = self.costmap.shape

        # Resolve costmap path
        script_directory = os.path.dirname(os.path.abspath(__file__))
        for _ in range(6):  # Move up 6 directories
            script_directory = os.path.dirname(script_directory)
        
        costmap_path = os.path.join(script_directory, "src", "nav_visualization", "costmaps", costmap_file)
        self.costmap = self.read_costmap(costmap_path)

        # Get window size parameters
        self.window_height = self.get_parameter('window_height').get_parameter_value().integer_value
        self.window_width = self.get_parameter('window_width').get_parameter_value().integer_value

        # Determine cell size based on provided window dimensions or screen size
        screen_width, screen_height = pyautogui.size()
        cell_width = (self.window_width or screen_width) / self.grid_width
        cell_height = (self.window_height or screen_height) / self.grid_height
        cell_size = int(min(cell_width, cell_height))

        self.window_height = self.grid_height * cell_size
        self.window_width = self.grid_width * cell_size
        self.cell_width = cell_size
        self.cell_height = cell_size

        # Initialize robot pose and path tracking
        self.robot_pose = self.start_position + [0.0]  # [x, y, theta]
        self.robot_path = [self.robot_pose[:2].copy()]
        self.twist_lock = threading.Lock()
        self.twist = Twist()

        # Initialize Pygame for visualization
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("ROS2 Costmap Visualization")

        # Subscribe to the Twist topic
        self.subscription = self.create_subscription(
            Twist, 
            topic,
            self.twist_callback,
            10
        )

        # Create a timer to update the visualization
        self.timer = self.create_timer(LocalPlanningVisualizer.DELTA_TIME, self.visualization_loop)

    def read_costmap(self, file_path):
        """
        Reads the costmap from a file.

        Args:
            file_path (str): Path to the costmap file.

        Returns:
            np.ndarray: The costmap as a NumPy array.
        """
        with open(file_path, 'r') as f:
            lines = f.readlines()
            self.start_position = list(map(int, lines[0].split()))
            self.goal_position = list(map(int, lines[1].split()))
            costmap = np.array([[int(num) for num in line.split()] for line in lines[2:]])
        return costmap

    def twist_callback(self, msg):
        """
        Updates the robot's velocity based on Twist messages.

        Args:
            msg (Twist): Twist message containing linear and angular velocity.
        """
        with self.twist_lock:
            self.twist = msg

    def draw_scene(self):
        """
        Draws the visualization including the costmap, start/goal positions, 
        the robot's path, and its current pose with velocity.
        """
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                cost = self.costmap[y, x]
                shade = 255 - int(255.0 / 100.0 * cost) if cost != -1 else 130
                color = (shade, shade, shade) if cost != -1 else (130, 0, 0)
                pygame.draw.rect(self.screen, color, 
                                 (x * self.cell_width, y * self.cell_height, self.cell_width, self.cell_height))

        # Draw start and goal points
        self.draw_circle(self.start_position, (0, 255, 0))  # Green for start
        self.draw_circle(self.goal_position, (0, 0, 255))    # Blue for goal

        # Update robot pose based on velocity
        with self.twist_lock:
            lin_vel_x, lin_vel_y, ang_vel = self.twist.linear.x, self.twist.linear.y, self.twist.angular.z

        x, y, theta = self.robot_pose
        x += LocalPlanningVisualizer.DELTA_TIME * lin_vel_x
        y += LocalPlanningVisualizer.DELTA_TIME * lin_vel_y
        theta += LocalPlanningVisualizer.DELTA_TIME * ang_vel
        self.robot_pose = [max(0, min(self.grid_width - 1, x)), max(0, min(self.grid_height - 1, y)), theta]

        # Append the current position to the robot's path
        self.robot_path.append([x, y])

        # Draw the robot's trajectory
        if len(self.robot_path) > 1:
            path_pixels = [(pt[0] * self.cell_width, pt[1] * self.cell_height) for pt in self.robot_path]
            pygame.draw.lines(self.screen, (255, 0, 0), False, path_pixels, 2)

        # Draw the robot's current position
        self.draw_circle(self.robot_pose[:2], (255, 0, 0))
        self.draw_robot_direction()

    def draw_circle(self, position, color):
        """
        Draws a circle at a given position.

        Args:
            position (list): [x, y] coordinates.
            color (tuple): RGB color.
        """
        center = (position[0] * self.cell_width + self.cell_width / 2, 
                  position[1] * self.cell_height + self.cell_height / 2)
        pygame.draw.circle(self.screen, color, center, min(self.cell_width, self.cell_height) / 3)

    def draw_robot_direction(self):
        """
        Draws an arrow indicating the robot's current orientation.
        """
        x, y, theta = self.robot_pose
        arrow_length = min(self.cell_width, self.cell_height)
        tip_x = x * self.cell_width + arrow_length * math.cos(theta)
        tip_y = y * self.cell_height + arrow_length * math.sin(theta)
        pygame.draw.line(self.screen, (0, 255, 0), (x * self.cell_width, y * self.cell_height), (tip_x, tip_y), 3)

    def visualization_loop(self):
        """
        Main visualization loop that handles events and updates the display.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_node()
                rclpy.shutdown()
                pygame.quit()
        
        self.screen.fill((0, 0, 0))
        self.draw_scene()
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    visualizer = LocalPlanningVisualizer()
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()