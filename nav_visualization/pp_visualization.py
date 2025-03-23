#!/usr/bin/env python3

"""
Path Planning Visualizer for ROS2

This script visualizes a robot's path planning process using pygame. It listens to a 
ROS2 action server (`navigate_to_goal`) and updates the robot's position in real-time 
based on feedback. The costmap is loaded from a text file, and the robot's movement 
towards the goal is visualized.

Features:
- Loads a costmap from a specified file.
- Sends a navigation goal to the ROS2 action server.
- Displays the costmap with start/goal positions.
- Draws the robot's trajectory and updates it in real-time.

Usage:
1. Run a ROS2 environment.
2. Execute this script.
3. The window will display the costmap and update the robot's position dynamically.
4. Close the pygame window to terminate the script.

Dependencies:
- ROS2 (rclpy)
- pygame
- numpy
- threading
- nav_msgs.msg.OccupancyGrid
- infra_interfaces.action.NavigateToGoal
- infra_interfaces.msg.CellCoordinateMsg
- std_msgs.msg.Header
"""

import rclpy
import pygame
import numpy as np
import os

from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from infra_interfaces.action import NavigateToGoal
from infra_interfaces.msg import CellCoordinateMsg
from std_msgs.msg import Header


class PathPlanningVisualizer(Node):
    """
    A ROS2 node that visualizes the robot's path planning process.

    This node:
    - Loads a costmap from a file.
    - Sends a goal to the `navigate_to_goal` action server.
    - Subscribes to feedback and updates the robot's position in real-time.
    - Uses pygame to render the costmap and robot trajectory.
    """

    def __init__(self):
        super().__init__('path_planning_visualizer')

        # Declare and retrieve parameters
        self.declare_parameter('costmap_file', 'costmap.txt')
        self.declare_parameter('window_height', None)
        self.declare_parameter('window_width', None)

        # Create an action client for path planning
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')

        # Start the navigation process
        self.send_goal()

    def variable_initiation(self, costmap_file):
        """
        Initializes variables including loading the costmap and setting up the visualization parameters.
        
        Args:
            costmap_file (str): The filename of the costmap.
        """

        # Resolve the costmap path by removing unnecessary directory levels
        script_directory = os.path.dirname(os.path.abspath(__file__))
        for _ in range(3):  # Move up 6 directories
            script_directory = os.path.dirname(script_directory)

        costmap_path = os.path.join(script_directory, "src", "nav_visualization", "costmaps", costmap_file)

        # Load costmap
        self.costmap = self.read_costmap(costmap_path)
        self.grid_height, self.grid_width = self.costmap.shape

        # Retrieve window dimensions
        self.window_height = self.get_parameter('window_height').get_parameter_value().integer_value
        self.window_width = self.get_parameter('window_width').get_parameter_value().integer_value

        # Determine cell size based on window or screen size
        screen_width = 800
        screen_height = 800
        cell_width = (self.window_width or screen_width) / self.grid_width
        cell_height = (self.window_height or screen_height) / self.grid_height
        self.cell_size = int(min(cell_width, cell_height))

        # Update window dimensions based on calculated cell size
        self.window_height = self.grid_height * self.cell_size
        self.window_width = self.grid_width * self.cell_size

        # Initialize robot position at the start
        self.robot_position = self.start_position.copy()
        # Track the robot's path from the start position
        self.robot_path = [self.robot_position.copy()]

    def grid_to_occupancy(self):
        """
        Converts the costmap into an OccupancyGrid message for ROS2.

        Returns:
            OccupancyGrid: The occupancy grid representation of the costmap.
        """
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = float(self.window_width / self.grid_width)
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0

        # Convert the grid into occupancy data
        msg.data = [int(cell) for row in self.costmap for cell in row]

        return msg

    def send_goal(self):
        """
        Sends the navigation goal to the ROS2 action server and starts the visualization process.
        """

        # Get costmap filename from parameters
        costmap_file = self.get_parameter('costmap_file').get_parameter_value().string_value
        self.variable_initiation(costmap_file)

        # Initialize Pygame for visualization
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("ROS2 Costmap Visualization")

        # Draw initial scene
        self.draw_scene()

        # Create goal message
        msg = NavigateToGoal.Goal()
        msg.start = CellCoordinateMsg(x=self.start_position[0], y=self.start_position[1])
        msg.goal = CellCoordinateMsg(x=self.goal_position[0], y=self.goal_position[1])
        msg.costmap = self.grid_to_occupancy()

        # Send goal to action server
        if not self._action_client.wait_for_server(timeout_sec = 5.0):
            raise Exception("Action server timed out - over 5 seconds of inactivity")
        self._send_goal_future = self._action_client.send_goal_async(msg, feedback_callback=self.feedback_position_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handles the response from the ROS2 action server.

        Args:
            future: The future object containing the goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handles the result from the ROS2 action server.

        Args:
            future: The future object containing the result.
        """
        result = future.result().result.success
        self.get_logger().info(f'Success state: {result}')
        rclpy.shutdown()

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
            # Flip costmap so that (0,0) is the bottom left corner
            costmap = np.flip(costmap, axis=0)
        return costmap

    def feedback_position_callback(self, feedback):
        """
        Handles position updates based on feedback from the action server.

        Args:
            feedback: The feedback message containing the robot's current position.
        """
        msg = feedback.feedback
        x, y = int(msg.distance_from_start.position.x), int(msg.distance_from_start.position.y)
        self.robot_position = [max(0, min(self.grid_width - 1, x)), max(0, min(self.grid_height - 1, y))]

        # Update robot path
        self.robot_path.append(self.robot_position.copy())

        # Redraw the scene
        self.draw_scene()

    def draw_scene(self):
        """
        Draws the visualization, including the costmap, start/goal positions, and robot trajectory.
        """
        self.screen.fill((0, 0, 0))

        cell_height = self.window_height / self.grid_height
        cell_width = self.window_width / self.grid_width
        radius = cell_height // 2

        # Draw costmap
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                cost = self.costmap[y, x]
                shade = 255 - int(255.0 / 100.0 * cost) if cost != -1 else 130
                color = (shade, shade, shade) if cost != -1 else (127, 0, 255)
                rect_y = (self.grid_height - 1 - y) * self.cell_size
                rect_x = x * self.cell_size
                pygame.draw.rect(self.screen, color, 
                                 (rect_x, rect_y, self.cell_size, self.cell_size))

        
        # Draw robot and path
        for point in self.robot_path:
            # Draw path as light green
            self.draw_circle(self.pose_to_pixel(point), (204, 255, 204))
        
        # Draw start and goal points
        self.draw_circle(self.pose_to_pixel(self.start_position), (0, 255, 0))  # Green for start
        self.draw_circle(self.pose_to_pixel(self.goal_position), (0, 0, 255))    # Blue for goal        

        # Draw robot
        self.draw_circle(self.pose_to_pixel(self.robot_position), (255, 0, 0))

        pygame.display.flip()

    def draw_circle(self, position, color):
            """
            Draws a circle at a given position on the screen.

            Args:
                position (list): [x, y] coordinates.
                color (tuple): RGB color.
            """
            pygame.draw.circle(self.screen, color, position, min(self.cell_size, self.cell_size) / 3)

    def pose_to_pixel(self, pose):
            """
            Converts a pose to pixel coordinates. For example, for a pose of [0,0],
            returns the pixel coordinates of the center of the costmap cell in the
            bottom left corner.

            Args:
                pose (list): [x, y] coordinates.

            Returns:
                tuple: (x, y) pixel coordinates.
            """
            return (pose[0] * self.cell_size + self.cell_size / 2, 
                (self.grid_height - 1 - pose[1]) * self.cell_size + self.cell_size / 2)


def main(args=None):
    rclpy.init(args=args)
    visualizer = PathPlanningVisualizer()
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