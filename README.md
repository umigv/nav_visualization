# nav_visualization

## draw_costmap.py
MAKE A COSTMAP:
Make a new terminal, go to your ws
cd ws
cd src
cd nav_visualization
python3 draw_costmap.py

draw your map
close the pygame window

### enabling your new map
put the file you created into the config folder
adjust the file name in pp_visualization or lp_visualization

add the following lines to setup.py
          # Config files
        (os.path.join('share', package_name, 'config'), 
         [os.path.join('config', 'YOUR FILE NAME')])


Open 3 terminals

terminal 1:
colcon build --packages-select nav_visualization nav_infrastructure

terminal 
source install/setup.zsh 
source install/setup.bash

run local visualizer:
ros2 run nav_visualization path_planning_visualizer

run path planning visualizer:
ros2 run nav_visualization local_planning_visualizer