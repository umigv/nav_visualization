#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import numpy as np
import threading
import math
class LocalPlanningVisualizer(Node):
    DELTA_TIME = 0.01
    
    def __init__(self):
        super().__init__('local_planning_visualizer')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('window_height', 600),
                ('window_width', 600),
                ('grid_height', 20),
                ('grid_width', 20),
                ('costmap_file', 'costmap.txt')
            ]
        )
        
        # Get parameters
        self.window_height = self.get_parameter('window_height').value
        self.window_width = self.get_parameter('window_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.grid_width = self.get_parameter('grid_width').value
        costmap_file = self.get_parameter('costmap_file').value

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
            '/robot_twist',
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


        # TODO draw a shape that has a direction
        center = (x * self.cell_width + self.cell_width/2, y * self.cell_height + self.cell_height/2)
        radius = min(self.cell_width, self.cell_height) / 3
        pygame.draw.circle(self.screen, (255, 0, 0), center, radius)
        self.draw_robot_direction()
        pygame.draw.rect(self.screen, "green", ((x-1) * self.cell_width, y * self.cell_height, self.cell_width, self.cell_height))
        pygame.draw.rect(self.screen, "green", ((x+1) * self.cell_width, y * self.cell_height, self.cell_width, self.cell_height))
        pygame.draw.rect(self.screen, "green", ((x) * self.cell_width, (y-1) * self.cell_height, self.cell_width, self.cell_height))
        pygame.draw.rect(self.screen, "green", ((x) * self.cell_width, (y+1) * self.cell_height, self.cell_width, self.cell_height))
        

    def draw_robot_direction(self, color=(0, 0, 255)):
      """Draws a robot as an arrow at a given position and angle."""

      arrow_length = min(self.cell_width, self.cell_height)
      # Calculate arrow tip (end point)
      x = self.robot_pose[0]
      y = self.robot_pose[1]
      theta = self.robot_pose[2]

      tip_x = x + arrow_length * math.cos(theta)
      tip_y = y + arrow_length * math.sin(theta)

      # Calculate arrowhead points
      left_wing = (tip_x - 15 * math.cos(theta - math.pi / 6), 
                  tip_y - 15 * math.sin(theta - math.pi / 6))
      right_wing = (tip_x - 15 * math.cos(theta + math.pi / 6), 
                    tip_y - 15 * math.sin(theta + math.pi / 6))

      # Draw main arrow line
      pygame.draw.line(self.screen, color, self.robot_pose[:2], (tip_x, tip_y), 5)

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