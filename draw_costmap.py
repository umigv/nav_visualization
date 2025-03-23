"""
Grid Drawing and Costmap Saving Tool

This script allows the user to create a grid-based costmap, set start and goal positions, 
and draw obstacles using mouse clicks. The final grid configuration is saved to a text file.

Usage:
1. Run the script.
2. Enter grid dimensions (width and height).
3. Enter the start and goal coordinates.
4. Enter a filename for saving the costmap.
5. A grid will be displayed where:
   - Left-clicking marks an obstacle (black).
   - Left-clicking again erases an obstacle (white).
6. Close the window to save the costmap.

Dependencies:
- pygame
- pyautogui
- os
- sys

The output costmap is saved in the "costmaps" directory, located one level up from the script directory.
"""

import pygame
import sys
import os

def create_grid(width, height):
    """
    Creates a 2D grid initialized with zeros.
    
    Args:
        width (int): Number of columns in the grid.
        height (int): Number of rows in the grid.

    Returns:
        list: 2D list representing the grid.
    """
    return [[0 for _ in range(width)] for _ in range(height)]

def save_costmap(start_x, start_y, goal_x, goal_y, grid, filename="costmap.txt"):
    """
    Saves the costmap configuration to a file.

    Args:
        start_x (int): X-coordinate of the start position.
        start_y (int): Y-coordinate of the start position.
        goal_x (int): X-coordinate of the goal position.
        goal_y (int): Y-coordinate of the goal position.
        grid (list): 2D list representing the grid.
        filename (str): Name of the file to save the costmap.
    """

    # Define save directory
    file_directory = os.path.dirname(os.path.abspath(__file__))
    parent_directory = os.path.dirname(file_directory)
    save_path = os.path.join(parent_directory, "costmaps", filename)

    # Ensure the costmaps directory exists
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    # Write the costmap to the file
    with open(save_path, 'w') as f:
        f.write(f"{start_x} {start_y}\n")
        f.write(f"{goal_x} {goal_y}\n")

        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")

    print(f"Costmap saved to {save_path}")

def draw_grid(screen, start_x, start_y, goal_x, goal_y, grid, cell_size):
    """
    Draws the grid on the pygame screen.

    Args:
        screen (pygame.Surface): The pygame window.
        start_x (int): X-coordinate of the start position.
        start_y (int): Y-coordinate of the start position.
        goal_x (int): X-coordinate of the goal position.
        goal_y (int): Y-coordinate of the goal position.
        grid (list): 2D list representing the grid.
        cell_size (int): Size of each grid cell.
    """

    for row_idx, row in enumerate(grid):
        for col_idx, cell in enumerate(row):
            color = (255, 255, 255) if cell == 0 else (0, 0, 0)  # White for empty, black for obstacles
            pygame.draw.rect(screen, color, (col_idx * cell_size, row_idx * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (200, 200, 200), (col_idx * cell_size, row_idx * cell_size, cell_size, cell_size), 1)  # Grid lines
    
    # Draw start position (Green)
    start_center = (start_x * cell_size + cell_size // 2, start_y * cell_size + cell_size // 2)
    pygame.draw.circle(screen, (0, 255, 0), start_center, min(cell_size, cell_size) // 3)

    # Draw goal position (Blue)
    goal_center = (goal_x * cell_size + cell_size // 2, goal_y * cell_size + cell_size // 2)
    pygame.draw.circle(screen, (0, 0, 255), goal_center, min(cell_size, cell_size) // 3)

def main():
    """
    Main function to handle user input, drawing the grid, and saving the costmap.
    """
    pygame.init()

    # Get user input for grid dimensions
    width = int(input("Enter grid width: "))
    height = int(input("Enter grid height: "))

    # Get start and goal coordinates
    start_x = int(input("Enter start x coordinate: "))
    start_y = int(input("Enter start y coordinate: "))
    goal_x = int(input("Enter goal x coordinate: "))
    goal_y = int(input("Enter goal y coordinate: "))

    # Get screen size and adjust cell size accordingly
    screen_width = 800
    screen_height = 800
    cell_width = screen_width / width
    cell_height = screen_height / height
    cell_size = int(min(cell_width, cell_height))

    # Ensure cell size is valid
    if cell_size < 1:
        raise ValueError("Grid is too large for the screen size.")

    # Get filename for saving the costmap
    filename = input("Enter filename to save to (include .txt in the filename): ")

    # Initialize pygame window
    screen = pygame.display.set_mode((width * cell_size, height * cell_size))
    pygame.display.set_caption("Draw on Grid")

    # Initialize grid and drawing flag
    grid = create_grid(width, height)
    drawing = False
    erasing = False

    running = True
    while running:
        screen.fill((255, 255, 255))
        draw_grid(screen, start_x, start_y, goal_x, goal_y, grid, cell_size)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                drawing = True
                x, y = pygame.mouse.get_pos()
                col, row = x // cell_size, y // cell_size
                if grid[row][col] == 100:
                    erasing = True  # If obstacle exists, enter erasing mode
                else:
                    erasing = False  # Otherwise, enter drawing mode

            elif event.type == pygame.MOUSEBUTTONUP:
                drawing = False

        # Handle drawing and erasing while mouse is pressed
        if drawing:
            x, y = pygame.mouse.get_pos()
            col, row = x // cell_size, y // cell_size

            if 0 <= row < height and 0 <= col < width:
                grid[row][col] = 0 if erasing else 100

    # Save costmap in the "costmaps" directory
    save_costmap(start_x, start_y, goal_x, goal_y, grid, filename)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()