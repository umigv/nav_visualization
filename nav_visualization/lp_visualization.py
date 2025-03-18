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
from infra_interfaces.action import NavigateToGoal
from infra_interfaces.msg import CellCoordinateMsg
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from rclpy.action import ActionClient
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
        self.declare_parameter('topic', '/cmd_vel')

        costmap_file = self.get_parameter('costmap_file').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Generate costmap path and load costmap
        script_directory = os.path.dirname(os.path.abspath(__file__))
        for _ in range(6):  # Move up 6 directories
            script_directory = os.path.dirname(script_directory)
        
        
        # costmap_path = os.path.join(script_directory, "src", "nav_visualization", "costmaps", costmap_file)
        costmap_path = '/home/arvuser/arv-ws/src/nav_visualization/costmaps/costmap3.txt'
        self.costmap = self.read_costmap(costmap_path)
        self.grid_height, self.grid_width = self.costmap.shape

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

        # Create an action client for path planning
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        print(f"twist topic: {topic}")
        # Subscribe to the Twist topic
        self.subscription = self.create_subscription(
            Twist, 
            topic,
            self.twist_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/odom',
            10
        )

        # Start the navigation process
        self.send_goal()

        # Create a timer to update the visualization
        self.timer = self.create_timer(LocalPlanningVisualizer.DELTA_TIME, self.visualization_loop)

    def send_goal(self):
        """
        Sends the navigation goal to the ROS2 action server and starts the visualization process.
        """

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
        rclpy.spin_until_future_complete(self, self._send_goal_future)

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
    
    def feedback_position_callback(self, feedback):
        """
        Handles position updates based on feedback from the action server.

        Args:
            feedback: The feedback message containing the robot's current position.
        """
        msg = feedback.feedback
        x, y = int(msg.distance_from_start.position.x), int(msg.distance_from_start.position.y)
        self.robot_position = [max(0, min(self.grid_width - 1, x)), max(0, min(self.grid_height - 1, y))]

        print("Current robot position: (" + str(self.robot_position[0]) + ", " + str(self.robot_position[1]) + ")")
    
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
                color = (shade, shade, shade) if cost != -1 else (127, 0, 255)
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

        # Publish the robot odometry 
        msg = Odometry()
        cov_pose = PoseWithCovariance()
        cov_pose.pose = self.robot_pose
        msg.pose = cov_pose
        msg.twist = self.twist

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing pose to /odom')
        
        # Append the current position to the robot's path
        self.robot_path.append([x, y])

        # Draw the robot's trajectory
        radius = self.cell_height // 2
        if len(self.robot_path) > 1:
            path_pixels = [(pt[0] * self.cell_width + radius, pt[1] * self.cell_height + radius) for pt in self.robot_path]
            pygame.draw.lines(self.screen, (204, 255, 204), False, path_pixels, 2)

        # Draw the robot's current position
        self.draw_circle(self.robot_pose[:2], (255, 0, 0))
        self.draw_robot_direction()
        self.draw_robot_velo()

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

    def draw_robot_direction(self, color=(0, 255, 0)):
      """
      Draws an arrow indicating the robot's current orientation.
      """
      arrow_length = min(self.cell_width, self.cell_height)
      # Calculate arrow tip (end point)
      radius = self.cell_height // 2
      x = self.robot_pose[0] * self.cell_width + radius
      y = self.robot_pose[1] * self.cell_height + radius
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

    def draw_robot_velo(self, color=(0, 0, 255)):
      """Draws a robot as an arrow at a given position and angle."""
      magnitude = int((((self.twist.linear.x ** 2 + self.twist.linear.y ** 2)) ** 0.5) * 5)
      arrow_length = min(self.cell_width, self.cell_height) * magnitude
      
      # Calculate arrow center
      radius = self.cell_height // 2
      x = self.robot_pose[0] * self.cell_width + radius
      y = self.robot_pose[1] * self.cell_height + radius

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