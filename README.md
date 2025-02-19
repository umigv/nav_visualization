# nav_visualization
# TODO - modify other launch files for correct parameter and command line reading in, and add window height and width as potential command line arguments
# TODO - add path tracing to both pp-visualization and ideally lp_visualization as well 
# TODO - complete documentation (add comments to files and properly complete readme)
# TODO - update requirements.txt

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
adjust the file name in pp_visualization or lp_visualization

add the following lines to setup.py
          # Config files
        (os.path.join('share', package_name, 'config'), 
         [os.path.join('config', 'YOUR FILE NAME')])


Open 2 terminals

terminal 1:
colcon build --packages-select nav_visualization nav_infrastructure

terminal 2:
source install/setup.zsh
or
source install/setup.bash

ros2 launch nav_visualization server-demo-pp.py