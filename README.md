# nav_visualization

## Overview
This repository provides tools for visualizing robot navigation on a grid-based costmap. It includes ROS2 launch files and utility scripts that allow you to create, simulate, and visualize both path and local planning demos.

## Launch Files

### server-demo-pp.py
- **Purpose:** Launches the path planning visualizer with the planner server.
- **Parameters:**
  - `costmap_file` (default: `costmap.txt`)
  - `window_width` (default: `800`)
  - `window_height` (default: `800`)
- **Usage Example:**
  ```
  ros2 launch nav_visualization server-demo-pp.py costmap_file:=your_costmap.txt window_width:=800 window_height:=600
  ```

### dummy-demo-pp.py
- **Purpose:** Runs the path planning visualizer with a dummy position publisher to simulate robot positions.
- **Parameter:**
  - `costmap_file` (default: `costmap.txt`)
- **Usage Example:**
  ```
  ros2 launch nav_visualization dummy-demo-pp.py costmap_file:=your_costmap.txt
  ```

### dummy-demo-lp.py
- **Purpose:** Launches the local planning visualizer with a dummy twist publisher to simulate real-time motion.
- **Parameters:**
  - `costmap_file` (default: `costmap.txt`)
  - `window_width` (default: `800`)
  - `window_height` (default: `800`)
  - `topic` (default: `/robot_twist`)
- **Usage Example:**
  ```
  ros2 launch nav_visualization dummy-demo-lp.py costmap_file:=your_costmap.txt window_width:=800 window_height:=600 topic:=/your_twist_topic
  ```

## Utility Scripts

### draw_costmap.py
Launches a pygame interface to create/edit costmaps. Follow the prompts to set grid dimensions, designate start/goal positions, and add or remove obstacles. The map is saved upon closing the window.

### pp_visualization.py
Visualizes the robot’s planned trajectory by reading a costmap and sending navigation goals using an action client. Window dimensions and costmap file can be modified via command line arguments.

### lp_visualization.py
Subscribes to a ROS2 Twist topic and renders the costmap along with real-time robot movement and direction. Includes adjustable parameters for file name, window size, and topic name.

### dummy_position_publisher.py
Simulates robot positions by reading the costmap file to determine the grid and publishing random, constrained positions based on the map’s boundaries.

### dummy_twist_publisher.py
Publishes simulated Twist messages at a fixed rate to mimic smooth, realistic robot motion. Parameters include the destination topic and update rate.

## Costmap File Format
- **Line 1:** Starting position coordinates (e.g., `1 1`).
- **Line 2:** Goal position coordinates (e.g., `7 7`).
- **Remaining Lines:** Rows of space-separated integers representing cell costs:
  - **0:** Free cell.
  - Negative numbers (e.g., `-1`) or high positive numbers (e.g., `100`) indicate obstacles or high-cost areas.

## Usage Instructions

1. **Creating a Costmap:**
   - Navigate to the repository directory:
     ```
     cd ws/src/nav_visualization
     python3 draw_costmap.py
     ```
   - Follow the on-screen instructions to define grid dimensions, start/goal positions, and obstacles. The costmap is saved after closing the interface.

2. **Integrating a Custom Costmap:**
   - Update the file name in either `pp_visualization.py` or `lp_visualization.py` by passing the new costmap file name as a parameter.
   - Optionally, add the new file in your configuration (e.g., in `setup.py`).

3. **Launching a Demo:**
   - **Build Packages:**
     ```
     colcon build --packages-select nav_visualization nav_infrastructure
     ```
   - **Source Setup File:**
     ```
     source install/setup.bash   # or source install/setup.zsh
     ```
   - **Launch Examples:**
     - **Path Planning Demo:**
       ```
       ros2 launch nav_visualization server-demo-pp.py costmap_file:=your_costmap.txt window_width:=800 window_height:=600
       ```
     - **Dummy Position Demo:**
       ```
       ros2 launch nav_visualization dummy-demo-pp.py costmap_file:=your_costmap.txt
       ```
     - **Local Planning Demo:**
       ```
       ros2 launch nav_visualization dummy-demo-lp.py costmap_file:=your_costmap.txt window_width:=800 window_height:=600 topic:=/your_twist_topic
       ```

