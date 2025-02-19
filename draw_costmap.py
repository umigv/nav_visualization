import pygame
import sys
import pyautogui
import os

def create_grid(width, height):
    return [[0 for _ in range(width)] for _ in range(height)]

def save_costmap(start_x, start_y, goal_x, goal_y, grid, filename="costmap.txt"):

    file_directory = os.path.dirname(os.path.abspath(__file__))
    file_directory = os.path.dirname(file_directory)
    save_path = os.path.join(file_directory, "costmaps", filename)

    with open(save_path, 'w') as f:
        f.write(" ".join(map(str,[start_x, start_y])) + "\n")
        f.write(" ".join(map(str,[goal_x, goal_y])) + "\n")

        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
    print(f"Costmap saved to {save_path}")

def draw_grid(screen, start_x, start_y, goal_x, goal_y, grid, cell_size):
    for row_idx, row in enumerate(grid):
        for col_idx, cell in enumerate(row):
            color = (255, 255, 255) if cell == 0 else (0, 0, 0)
            pygame.draw.rect(screen, color, (col_idx * cell_size, row_idx * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (200, 200, 200), (col_idx * cell_size, row_idx * cell_size, cell_size, cell_size), 1)
    

        start_center = (start_x * cell_size + cell_size / 2, 
                        start_y * cell_size + cell_size / 2)
        goal_center = (goal_x * cell_size + cell_size / 2, 
                       goal_y * cell_size + cell_size / 2)
        pygame.draw.circle(screen, (0, 255, 0), start_center, min(cell_size, cell_size) / 3)  # Green for start
        pygame.draw.circle(screen, (0, 0, 255), goal_center, min(cell_size, cell_size) / 3)  # Blue for goal


def main():
    pygame.init()

    width = int(input("Enter grid width: "))
    height = int(input("Enter grid height: "))

    start_x = int(input("Enter start x coordinate: "))
    start_y = int(input("Enter start y coordinate: "))
    goal_x = int(input("Enter goal x coordinate: "))
    goal_y = int(input("Enter goal y coordinate: "))

    screen_width, screen_height = pyautogui.size()
    cell_width = screen_width / width
    cell_height = screen_height / height
    cell_size = int(min(cell_width, cell_height))
    
    if cell_size < 1:
        assert ValueError("Window is too big for screen size")

    filename = input("Enter filename to save to (include .txt in the filename): ")

    screen = pygame.display.set_mode((width * cell_size, height * cell_size))
    pygame.display.set_caption("Draw on Grid")

    grid = create_grid(width, height)
    drawing = False
    
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
                    erasing = True         
                else:
                    erasing = False
                
            elif event.type == pygame.MOUSEBUTTONUP:
                drawing = False
            
        if drawing:
            x, y = pygame.mouse.get_pos()
            
            col, row = x // cell_size, y // cell_size

            if 0 <= row < height and 0 <= col < width:
                if erasing:
                    grid[row][col] = 0
                else:
                    grid[row][col] = 100

    # Append config directory to filename
    filename  = "config/" + filename

    save_costmap(start_x, start_y, goal_x, goal_y, grid, filename)
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
