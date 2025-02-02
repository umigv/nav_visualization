#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import numpy as np
import threading
import math
from ament_index_python.packages import get_package_share_directory
import os



# This should be set to the topic that the path planning node is publishing to, 
# so that it can be visualized
TOPIC = '/robot_twist'

#parameters
WINDOW_HEIGHT = 600
WINDOW_WIDTH = 600
GRID_HEIGHT = 20
GRID_WIDTH = 20
COSTMAP_FILE = 'costmap.txt'

#dont change these
pkg_dir = get_package_share_directory('nav_visualization')
COSTMAP = os.path.join(pkg_dir, 'config', COSTMAP_FILE)

class LocalPlanningVisualizer(Node):
    DELTA_TIME = 0.01
    
    def __init__(self):
        super().__init__('local_planning_visualizer')
        
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

        self.cell_height = self.window_height / self.grid_height
        self.cell_width = self.window_width / self.grid_width
        
        # Load costmap
        self.costmap = self.read_costmap(costmap_file)
        
        # Verify costmap dimensions
        if self.costmap.shape != (self.grid_height, self.grid_width):
            self.get_logger().error("Costmap dimensions don't match grid parameters!")
            raise ValueError("Costmap dimensions mismatch")

        # Initialize robot position
        self.robot_pose = [self.grid_width / 2, self.grid_height / 2, 0]
        self.twist_lock = threading.Lock()
        self.twist = Twist()

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("ROS2 Costmap Visualization")

        # Create subscriber
        self.subscription = self.create_subscription(
            Twist, 
            TOPIC,
            self.twist_callback,
            10)
        
        # Create timer for visualization update
        self.timer = self.create_timer(LocalPlanningVisualizer.DELTA_TIME, self.visualization_loop)

    def read_costmap(self, file_path):
        """Read costmap from file"""
        with open(file_path, 'r') as f:
            return np.array([[int(num) for num in line.split()] for line in f])

    def twist_callback(self, msg):
        """Handle position updates"""
        with self.twist_lock:
            self.twist = msg
        

    def draw_scene(self):
        """Draw the visualization"""
        
        # Draw costmap
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                cost = self.costmap[y, x]
                color = (cost, cost, cost)
                pygame.draw.rect(self.screen, color,
                               (x * self.cell_width, y * self.cell_height, self.cell_width, self.cell_height))
        
        # Draw robot
        with self.twist_lock:
            lin_vel_x = self.twist.linear.x
            lin_vel_y = self.twist.linear.y
            ang_vel = self.twist.angular.z


        x, y, theta = self.robot_pose
        x += LocalPlanningVisualizer.DELTA_TIME * lin_vel_x
        y += LocalPlanningVisualizer.DELTA_TIME * lin_vel_y
        theta += LocalPlanningVisualizer.DELTA_TIME * ang_vel
        self.robot_pose = [x, y, theta]
        x = max(0, min(self.grid_width - 1, x))
        y = max(0, min(self.grid_height - 1, y))


        
        center = (x * self.cell_width, y * self.cell_height)
        radius = min(self.cell_width, self.cell_height) // 3
        pygame.draw.circle(self.screen, (255, 0, 0), center, radius)
        self.draw_robot_direction()
        self.draw_robot_velo()
        

    def draw_robot_direction(self, color=(0, 255, 0)):
      """Draws a robot as an arrow at a given position and angle."""
      arrow_length = min(self.cell_width, self.cell_height)
      # Calculate arrow tip (end point)
      x = self.robot_pose[0] * self.cell_width 
      y = self.robot_pose[1] * self.cell_height
      theta = self.robot_pose[2]

      tip_x = x + arrow_length * math.cos(theta)
      tip_y = y + arrow_length * math.sin(theta)
      wing_size = min(self.cell_width, self.cell_height) // 3
      arrow_line_width = max(int(min(self.cell_width, self.cell_height) / 9), 1)


      # Calculate arrowhead points
      left_wing = (tip_x - wing_size * math.cos(theta - math.pi / 6), 
                  tip_y - wing_size * math.sin(theta - math.pi / 6))
      right_wing = (tip_x - wing_size * math.cos(theta + math.pi / 6), 
                    tip_y - wing_size * math.sin(theta + math.pi / 6))

      # Draw main arrow line
      pygame.draw.line(self.screen, color, [x,y], (tip_x, tip_y), arrow_line_width)

      # Draw arrowhead
      pygame.draw.polygon(self.screen, color, [left_wing, (tip_x, tip_y), right_wing])

    def draw_robot_velo(self, color=(230, 255, 0)):
      """Draws a robot as an arrow at a given position and angle."""
      magnitude = int((((self.twist.linear.x ** 2 + self.twist.linear.y ** 2)) ** 0.5) * 3)
      arrow_length = min(self.cell_width, self.cell_height) * magnitude
      
      # Calculate arrow center
      x = self.robot_pose[0] * self.cell_width 
      y = self.robot_pose[1] * self.cell_height

      theta = math.atan(self.twist.linear.y / (self.twist.linear.x + 0.00001))

      if (theta > 0 and self.twist.linear.y < 0):
          theta += math.pi

      if (theta < 0 and self.twist.linear.x < 0):
          theta += math.pi

      # Calculate arrow tip (end point)
      tip_x = x + arrow_length * math.cos(theta) 
      tip_y = y + arrow_length * math.sin(theta) 
      wing_size = min(self.cell_width, self.cell_height) // 3
      arrow_line_width = max(int(min(self.cell_width, self.cell_height) / 9), 1)


      # Calculate arrowhead points
      left_wing = (tip_x - wing_size * math.cos(theta - math.pi / 6), 
                  tip_y - wing_size * math.sin(theta - math.pi / 6))
      right_wing = (tip_x - wing_size * math.cos(theta + math.pi / 6), 
                    tip_y - wing_size * math.sin(theta + math.pi / 6))

      # Draw main arrow line
      pygame.draw.line(self.screen, color, [x,y], (tip_x, tip_y), arrow_line_width)

      # Draw arrowhead
      pygame.draw.polygon(self.screen, color, [left_wing, (tip_x, tip_y), right_wing])

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