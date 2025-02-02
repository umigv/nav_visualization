# nav_visualization

to compile:
colcon build --packages-select nav_visualization

source:
source install/setup.zsh 
source install/setup.bash

run local visualizer:
ros2 launch nav_visualization demo.launch2.py

run path planning visualizer
ros2 launch nav_visualization demo.launch.py