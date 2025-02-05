import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import pygame

TOPIC = '/custom_grid'

class GridPublisher(Node):
    def __init__(self, grid_width, grid_height, cell_size):
        super().__init__('grid_publisher')
        
        # Create publisher for OccupancyGrid
        self.publisher_ = self.create_publisher(OccupancyGrid, TOPIC, 10)
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.cell_size = cell_size

        # Initialize empty grid with all cells free (255 for white)
        self.grid = [[255 for _ in range(grid_width)] for _ in range(grid_height)]
        self.drawing = False

        # Initialize pygame screen
        pygame.init()
        self.screen = pygame.display.set_mode((self.grid_width * self.cell_size, self.grid_height * self.cell_size))
        pygame.display.set_caption("Draw on Grid for ROS2 OccupancyGrid Publisher")
        # self.timer = self.create_timer(0.1, self.publish_grid)

        # Timer for periodic grid updates
        self.timer = self.create_timer(0.1, self.timer_callback)  # Call timer_callback every 0.1 seconds


    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = float(self.cell_size)
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0

        # Convert grid to occupancy values for ROS (255 -> -1 unknown, 0 -> free, 100 -> occupied)
        data = []
        for row in self.grid:
            for cell in row:
                if cell == 255:  # Free
                    data.append(0)
                else:  # Occupied
                    data.append(100)

        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info("Published OccupancyGrid message")
        

    def draw_grid(self):
        # Draw the entire grid in the Pygame window
        self.screen.fill((255, 255, 255))
        for row_idx, row in enumerate(self.grid):
            for col_idx, cell in enumerate(row):
                color = (255, 255, 255) if cell == 255 else (0, 0, 0)
                pygame.draw.rect(self.screen, color, (col_idx * self.cell_size, row_idx * self.cell_size, self.cell_size, self.cell_size))
                pygame.draw.rect(self.screen, (200, 200, 200), (col_idx * self.cell_size, row_idx * self.cell_size, self.cell_size, self.cell_size), 1)

        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Exiting program and publishing grid.")
                self.publish_grid()
                rclpy.shutdown()
                pygame.quit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.drawing = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.drawing = False

        if self.drawing:
            x, y = pygame.mouse.get_pos()
            col, row = x // self.cell_size, y // self.cell_size
            if 0 <= row < self.grid_height and 0 <= col < self.grid_width:
                self.grid[row][col] = 0
    def timer_callback(self):
        self.draw_grid()


def main(args=None):
    rclpy.init(args=args)
    
    width = int(input("Enter grid width: "))
    height = int(input("Enter grid height: "))
    cell_size = int(input("Enter cell size (in pixels): "))

    node = GridPublisher(width, height, cell_size)
    rclpy.spin(node)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
    #pygame.quit()


if __name__ == '__main__':
    main()
