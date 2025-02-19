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
import pyautogui


# This should be set to the topic that the path planning node is publishing to, 
# so that it can be visualized
TOPIC = '/robot_twist'

#parameters
WINDOW_HEIGHT = None
WINDOW_WIDTH = None
COSTMAP_FILE = 'costmap3.txt'

#dont change these
pkg_dir = get_package_share_directory('nav_visualization')
COSTMAP = os.path.join(pkg_dir, 'config', COSTMAP_FILE)

class LocalPlanningVisualizer(Node):
    DELTA_TIME = 0.01
    
    def __init__(self):
        super().__init__('local_planning_visualizer')
        
        self.declare_parameter('costmap_file', 'costmap.txt')

        # Get parameters
        costmap_file = self.get_parameter('costmap_file').get_parameter_value().string_value

        
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

        self.cell_height = self.window_height / self.grid_height
        self.cell_width = self.window_width / self.grid_width

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
            f = f.readlines()
            coords = f[:2]
            grid = f[2:]

            self.start_position = list(map(int, coords[0].split()))
            self.goal_position = list(map(int, coords[1].split()))

            costmap = np.array([[int(num) for num in line.split()] for line in grid])

            return costmap


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
                if cost == -1:
                    color = (130, 0, 0)
                else: 
                    shade = 255 - int(255.0/100.0 * cost)
                    color = (shade, shade, shade)
                pygame.draw.rect(self.screen, color,
                               (x * self.cell_width, y * self.cell_height, self.cell_width, self.cell_height))
        
        start_center = (self.start_position[0] * self.cell_width + self.cell_width / 2, 
                        self.start_position[1] * self.cell_height + self.cell_height / 2)
        goal_center = (self.goal_position[0] * self.cell_width + self.cell_width / 2, 
                       self.goal_position[1] * self.cell_height + self.cell_height / 2)
        pygame.draw.circle(self.screen, (0, 255, 0), start_center, min(self.cell_width, self.cell_height) / 3)  # Green for start
        pygame.draw.circle(self.screen, (0, 0, 255), goal_center, min(self.cell_width, self.cell_height) / 3)  # Blue for goal

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
      magnitude = int((((self.twist.linear.x ** 2 + self.twist.linear.y ** 2)) ** 0.5) * 5)
      arrow_length = min(self.cell_width, self.cell_height) * magnitude
      
      # Calculate arrow center
      x = self.robot_pose[0] * self.cell_width 
      y = self.robot_pose[1] * self.cell_height

      if self.twist.linear.x == 0:
          theta = math.atan(self.twist.linear.y / (self.twist.linear.x + 0.00001))
      else: 
          theta = math.atan(self.twist.linear.y / (self.twist.linear.x))

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