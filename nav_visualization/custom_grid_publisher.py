import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import pygame
import threading

TOPIC = '/custom_grid'

class GridPublisher(Node):
    def __init__(self, grid_width, grid_height, cell_size, start, goal):
        super().__init__('grid_publisher')
        
        # Create publisher for OccupancyGrid
        self.publisher_ = self.create_publisher(OccupancyGrid, TOPIC, 10)
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.cell_size = cell_size
        self.start = start
        self.goal = goal

        # Flags to track drawing and publishing
        self.complete = False    # True when drawing is done (e.g., window closed)
        self.published = False   # True once we have published the grid

        # Create a grid with all cells free (255 = white)
        self.grid = [[255 for _ in range(grid_width)] for _ in range(grid_height)]
        self.drawing = False

        # Initialize pygame screen
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.grid_width * self.cell_size, self.grid_height * self.cell_size)
        )
        pygame.display.set_caption("Draw on Grid for ROS2 OccupancyGrid Publisher")

        # Use a timer callback to update the display and check for publishing.
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.complete:
            if not self.published:
                self.publish_grid()
                self.published = True
                self.get_logger().info("Published occupancy grid. Preparing to shut down...")
            pygame.event.post(pygame.event.Event(pygame.QUIT))
            self.timer.cancel()
            threading.Thread(target=self.shutdown_ros).start()
        else:
            self.draw_grid()

    def shutdown_ros(self):
        rclpy.shutdown()

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

        # Convert the grid into occupancy data:
        data = []
        for row in self.grid:
            for cell in row:
                data.append(0 if cell == 255 else 100)
        msg.data = data

        self.publisher_.publish(msg)
        self.get_logger().info("OccupancyGrid published!")

    def draw_grid(self):
        # Clear the screen and draw the grid
        self.screen.fill((255, 255, 255))
        for row_idx, row in enumerate(self.grid):
            for col_idx, cell in enumerate(row):
                color = (255, 255, 255) if cell == 255 else (0, 0, 0)
                pygame.draw.rect(
                    self.screen, color,
                    (col_idx * self.cell_size, row_idx * self.cell_size,
                     self.cell_size, self.cell_size)
                )
                pygame.draw.rect(
                    self.screen, (200, 200, 200),
                    (col_idx * self.cell_size, row_idx * self.cell_size,
                     self.cell_size, self.cell_size), 1
                )
        # Draw start and goal points
        if self.start:
            pygame.draw.rect(self.screen, (0, 255, 0),
                              (self.start[0] * self.cell_size, self.start[1] * self.cell_size,
                               self.cell_size, self.cell_size))
        if self.goal:
            pygame.draw.rect(self.screen, (255, 0, 0),
                              (self.goal[0] * self.cell_size, self.goal[1] * self.cell_size,
                               self.cell_size, self.cell_size))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.complete = True
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.drawing = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.drawing = False

        if self.drawing:
            x, y = pygame.mouse.get_pos()
            col, row = x // self.cell_size, y // self.cell_size
            if 0 <= row < self.grid_height and 0 <= col < self.grid_width:
                if self.grid[row][col] == 0:
                    self.grid[row][col] = 255
                else:
                    self.grid[row][col] = 0


def main(args=None):
    rclpy.init(args=args)

    width = int(input("Enter grid width: "))
    height = int(input("Enter grid height: "))
    cell_size = int(input("Enter cell size (in pixels): "))

    start_x = int(input("Enter start x coordinate: "))
    start_y = int(input("Enter start y coordinate: "))
    goal_x = int(input("Enter goal x coordinate: "))
    goal_y = int(input("Enter goal y coordinate: "))

    start = (start_x, start_y)
    goal = (goal_x, goal_y)

    node = GridPublisher(width, height, cell_size, start, goal)
    rclpy.spin(node)

    print("Destroying node...")
    node.destroy_node()
    print("Quitting pygame...")
    pygame.quit()

if __name__ == '__main__':
    main()
