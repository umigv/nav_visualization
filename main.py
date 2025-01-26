import pygame
import numpy as np
import random

# Initialize Pygame
pygame.init()

# Constants
WINDOW_SIZE = 600  # Window size in pixels
GRID_SIZE = 20     # Number of grid cells per row/column
CELL_SIZE = WINDOW_SIZE // GRID_SIZE  # Size of each cell in pixels

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
ROBOT_COLOR = (255, 0, 0)  # Red for the robot

# Generate a random costmap (values between 0 and 255)
costmap = np.random.randint(0, 256, (GRID_SIZE, GRID_SIZE))

# Initialize the robot's position
robot_position = [GRID_SIZE // 2, GRID_SIZE // 2]  # Start at the center

# Create the display
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Costmap Visualization")

# Clock for controlling the frame rate
clock = pygame.time.Clock()

# Function to draw the costmap
def draw_costmap():
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            cost = costmap[y, x]
            color = (cost, cost, cost)  # Grayscale color based on cost
            pygame.draw.rect(screen, color, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

# Function to draw the robot
def draw_robot():
    x, y = robot_position
    pygame.draw.circle(screen, ROBOT_COLOR, (x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 3)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update robot position (random walk for demonstration)
    robot_position[0] += random.choice([-1, 0, 1])
    robot_position[1] += random.choice([-1, 0, 1])

    # Keep the robot within bounds
    robot_position[0] = max(0, min(GRID_SIZE - 1, robot_position[0]))
    robot_position[1] = max(0, min(GRID_SIZE - 1, robot_position[1]))

    # Clear the screen
    screen.fill(BLACK)

    # Draw the costmap and robot
    draw_costmap()
    draw_robot()

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    clock.tick(10)

# Quit Pygame
pygame.quit()
