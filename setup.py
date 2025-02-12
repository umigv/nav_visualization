from setuptools import setup
import os

package_name = 'nav_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Resource file
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        
        # Package.xml
        (os.path.join('share', package_name), ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', 'demo.launch.py')]),

        # Launch 2 files
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', 'demo.launch2.py')]),
        
        # Config files
        (os.path.join('share', package_name, 'config'), 
         [os.path.join('config', 'costmap.txt')]),

         # Config files
        (os.path.join('share', package_name, 'config'), 
         [os.path.join('config', 'costmap2.txt')]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Navigation Visualization Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'path_planning_visualizer = nav_visualization.pp_visualization:main',
            'local_planning_visualizer = nav_visualization.lp_visualization:main',
            'dummy_position_publisher = nav_visualization.dummy_position_publisher:main',
            'dummy_twist_publisher = nav_visualization.dummy_twist_publisher:main',
            'grid_publisher = nav_visualization.custom_grid_publisher:main'
        ],
    },
)