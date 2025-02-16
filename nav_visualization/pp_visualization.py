#!/usr/bin/env python3

import rclpy
import pygame
import numpy as np
import threading
import os
import pyautogui

from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from infra_interfaces.action import NavigateToGoal
from infra_interfaces.msg import Coordinate2D
from std_msgs.msg import Header

# Parameters
# Set these to none if for auto sizing
WINDOW_HEIGHT = None
WINDOW_WIDTH = None
COSTMAP_FILE = 'costmap3.txt'

# Don't change these
pkg_dir = get_package_share_directory('nav_visualization')
COSTMAP = os.path.join(pkg_dir, 'config', COSTMAP_FILE)

class PathPlanningVisualizer(Node):
    def __init__(self):
        super().__init__('path_planning_visualizer')
        
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        
        # Get parameters
        costmap_file = COSTMAP

        # Load costmap
        self.costmap = self.read_costmap(costmap_file)
        self.grid_height, self.grid_width = self.costmap.shape

        if (not WINDOW_HEIGHT) or (not WINDOW_WIDTH):
            screen_width, screen_height = pyautogui.size()
            cell_width = screen_width / self.grid_width
            cell_height = screen_height / self.grid_height
            cell_size = int(min(cell_width, cell_height))

            self.window_height = self.grid_height * cell_size
            self.window_width = self.grid_width * cell_size
        else: 
            self.window_height = WINDOW_HEIGHT
            self.window_width = WINDOW_WIDTH
            cell_width = WINDOW_WIDTH / self.grid_width
            cell_height = WINDOW_HEIGHT / self.grid_height
        # Initialize robot position, start, and goal positions
        self.robot_position = self.start_position

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("ROS2 Costmap Visualization")

        # Draw the initial scene before sending costmap and starting navigation process
        self.draw_scene()

        #self.create_timer(0.1, self.send_goal)
        self.send_goal()

    def grid_to_occupancy(self):
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

        # Convert the grid into occupancy data:
        data = []
        for row in self.costmap:
            for cell in row:
                data.append(0 if cell == 255 else 100)
        msg.data = data

        return msg
    
    def send_goal(self):
        msg = NavigateToGoal.Goal()
        self.get_logger().info('navigation started')
        msg.goal = Coordinate2D()
        msg.goal.x = self.goal_position[0]
        msg.goal.y = self.goal_position[1]

        self.get_logger().info('message created')

        msg.costmap = self.grid_to_occupancy()
        #self.get_logger().info(f'{msg}')
        self.get_logger().info('waiting for server')

        self._action_client.wait_for_server()

        self.get_logger().info('sending goal')  

        self._send_goal_future = self._action_client.send_goal_async(msg, feedback_callback=self.feedback_position_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('success')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().success
        self.get_logger().info('Success state: {0}'.format(result))
        rclpy.shutdown()
    
    def read_costmap(self, file_path):
        """Read costmap from file"""
        with open(file_path, 'r') as f:
            f = f.readlines()
            coords = f[:2]
            grid = f[2:]


            self.start_position = list(map(int, coords[0].split()))
            self.goal_position = list(map(int, coords[1].split()))

            costmap = np.array([[int(num) for num in line.split()] for line in grid])

            return costmap

    def feedback_position_callback(self, feedback):
        
        msg = feedback.feedback
        self.get_logger().info('Received feedback')

        """Handle position updates"""
        x = int(msg.distance_from_start.position.x)
        y = int(msg.distance_from_start.position.y)
        x = max(0, min(self.grid_width - 1, x))
        y = max(0, min(self.grid_height - 1, y))
        self.robot_position = [x, y]

        self.draw_scene()

    def draw_scene(self):
        """Draw the visualization"""
                
        self.screen.fill((0, 0, 0))

        cell_height = self.window_height / self.grid_height
        cell_width = self.window_width / self.grid_width

        # Draw costmap
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                cost = self.costmap[y, x]
                color = (cost, cost, cost)
                pygame.draw.rect(self.screen, color,
                                  (x * cell_width, y * cell_height, cell_width, cell_height))

        # Draw start and goal points
        start_center = (self.start_position[0] * cell_width + cell_width / 2, 
                        self.start_position[1] * cell_height + cell_height / 2)
        goal_center = (self.goal_position[0] * cell_width + cell_width / 2, 
                       self.goal_position[1] * cell_height + cell_height / 2)
        pygame.draw.circle(self.screen, (0, 255, 0), start_center, min(cell_width, cell_height) / 3)  # Green for start
        pygame.draw.circle(self.screen, (0, 0, 255), goal_center, min(cell_width, cell_height) / 3)  # Blue for goal

        # Draw robot
        x, y = self.robot_position
        center = (x * cell_width + cell_width / 2, y * cell_height + cell_height / 2)
        radius = min(cell_width, cell_height) / 3
        pygame.draw.circle(self.screen, (255, 0, 0), center, radius)  # Red for robot

        pygame.display.flip()

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
