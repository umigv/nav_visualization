import pygame
import sys

def create_grid(width, height, cell_size):
    return [[0 for _ in range(width)] for _ in range(height)]

def save_costmap(grid, filename="costmap.txt"):
    with open(filename, 'w') as f:
        for row in grid:
            f.write(" ".join(map(str, row)) + "\n")
    print(f"Costmap saved to {filename}")

def draw_grid(screen, grid, cell_size):
    for row_idx, row in enumerate(grid):
        for col_idx, cell in enumerate(row):
            color = (255, 255, 255) if cell == 0 else (0, 0, 0)
            pygame.draw.rect(screen, color, (col_idx * cell_size, row_idx * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (200, 200, 200), (col_idx * cell_size, row_idx * cell_size, cell_size, cell_size), 1)

def main():
    pygame.init()

    width = int(input("Enter grid width: "))
    height = int(input("Enter grid height: "))
    cell_size = 20  # Size of each cell in pixels

    screen = pygame.display.set_mode((width * cell_size, height * cell_size))
    pygame.display.set_caption("Draw on Grid")

    grid = create_grid(width, height, cell_size)
    drawing = False
    
    running = True
    while running:
        screen.fill((255, 255, 255))
        draw_grid(screen, grid, cell_size)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                drawing = True
            elif event.type == pygame.MOUSEBUTTONUP:
                drawing = False
            
        if drawing:
            x, y = pygame.mouse.get_pos()
            col, row = x // cell_size, y // cell_size
            if 0 <= row < height and 0 <= col < width:
                grid[row][col] = 255

    save_costmap(grid)
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
