#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pygame
import numpy as np
import threading
from ament_index_python.packages import get_package_share_directory
import os

# This should be set to the topic that the path planning node is publishing to, 
# so that it can be visualized
TOPIC = '/robot_position'

#parameters
WINDOW_HEIGHT = 600
WINDOW_WIDTH = 600
GRID_HEIGHT = 20
GRID_WIDTH = 20
COSTMAP_FILE = 'costmap.txt'


# dont change these
pkg_dir = get_package_share_directory('nav_visualization')
COSTMAP = os.path.join(pkg_dir, 'config', COSTMAP_FILE)

class PathPlanningVisualizer(Node):
    def __init__(self):
        super().__init__('path_planning_visualizer')
        
        # # Declare parameters
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('window_height', 600),
        #         ('window_width', 600),
        #         ('grid_height', 20),
        #         ('grid_width', 20),
        #         ('costmap_file', 'costmap.txt')
        #     ]
        # )
        
        # Get parameters
        self.window_height = WINDOW_HEIGHT
        self.window_width = WINDOW_WIDTH
        self.grid_height = GRID_HEIGHT
        self.grid_width = GRID_WIDTH
        costmap_file = COSTMAP
        
        # Load costmap
        self.costmap = self.read_costmap(costmap_file)
        
        # Verify costmap dimensions
        if self.costmap.shape != (self.grid_height, self.grid_width):
            self.get_logger().error("Costmap dimensions don't match grid parameters!")
            raise ValueError("Costmap dimensions mismatch")

        # Initialize robot position
        self.robot_position = [self.grid_width // 2, self.grid_height // 2]
        self.position_lock = threading.Lock()

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("ROS2 Costmap Visualization")

        # Create subscriber
        self.subscription = self.create_subscription(
            Point,
            TOPIC,
            self.position_callback,
            10)
        
        # Create timer for visualization update
        self.timer = self.create_timer(0.1, self.visualization_loop)

    def read_costmap(self, file_path):
        """Read costmap from file"""
        with open(file_path, 'r') as f:
            return np.array([[int(num) for num in line.split()] for line in f])

    def position_callback(self, msg):
        """Handle position updates"""
        x = int(msg.x)
        y = int(msg.y)
        x = max(0, min(self.grid_width - 1, x))
        y = max(0, min(self.grid_height - 1, y))
        with self.position_lock:
            self.robot_position = [x, y]

    def draw_scene(self):
        """Draw the visualization"""
        cell_height = self.window_height / self.grid_height
        cell_width = self.window_width / self.grid_width
        
        # Draw costmap
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                cost = self.costmap[y, x]
                color = (cost, cost, cost)
                pygame.draw.rect(self.screen, color,
                               (x * cell_width, y * cell_height, cell_width, cell_height))
        
        # Draw robot
        with self.position_lock:
            x, y = self.robot_position
        center = (x * cell_width + cell_width/2, y * cell_height + cell_height/2)
        radius = min(cell_width, cell_height) / 3
        pygame.draw.circle(self.screen, (255, 0, 0), center, radius)
        pygame.draw.rect(self.screen, "green", ((x-1) * cell_width, y * cell_height, cell_width, cell_height))
        pygame.draw.rect(self.screen, "green", ((x+1) * cell_width, y * cell_height, cell_width, cell_height))
        pygame.draw.rect(self.screen, "green", ((x) * cell_width, (y-1) * cell_height, cell_width, cell_height))
        pygame.draw.rect(self.screen, "green", ((x) * cell_width, (y+1) * cell_height, cell_width, cell_height))
        

    def visualization_loop(self):
        """Main visualization loop"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.destroy_node()
                rclpy.shutdown()
        
        self.screen.fill((0, 0, 0))
        self.draw_scene()
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