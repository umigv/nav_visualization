# nav_visualization

MAKE A COSTMAP:
run draw_costmap.py
close the window
put the file you created into the config folder
adjust the file name in pp_visualization or lp_visualization

add the following lines to setup.py
         # Config files
        (os.path.join('share', package_name, 'config'), 
         [os.path.join('config', 'YOUR FILE NAME')])

to compile:
colcon build --packages-select nav_visualization

source:
source install/setup.zsh 
source install/setup.bash

run local visualizer:
ros2 run nav_visualization path_planning_visualizer

run path planning visualizer:
ros2 run nav_visualization local_planning_visualizer