#!/usr/bin/env python3
"""
Navigation Visualization Package Setup

This setup file is used to package and install the 'nav_visualization' ROS2 package.
It defines package metadata, resource files, and entry points for console scripts.

Usage:
    To install the package, run:
        python3 setup.py install

    To develop (editable install):
        pip install -e .

Dependencies:
    - setuptools

Make sure that your package structure follows the ROS2 conventions with the appropriate
files in the 'resource', 'launch', and package directories.
"""

from setuptools import setup
import os

# Define the package name
package_name = 'nav_visualization'

# Setup configuration
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Resource file for ament index (used by ROS2 to locate packages)
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        
        # Package manifest
        (os.path.join('share', package_name), ['package.xml']),
        
        # Launch file for dummy twist publisher demo for local planning
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', 'dummy-demo-lp.py')]),

        # Launch file for dummy position publisher demo for path planning
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', 'dummy-demo-pp.py')]),
        
        # Launch file for action server demo for path planning
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', 'server-demo-pp.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Navigation Visualization Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # Entry points for executable scripts (ensure these modules have a main() function)
            'path_planning_visualizer = nav_visualization.pp_visualization:main',
            'local_planning_visualizer = nav_visualization.lp_visualization:main',
            'dummy_position_publisher = nav_visualization.dummy_position_publisher:main',
            'dummy_twist_publisher = nav_visualization.dummy_twist_publisher:main',
        ],
    },
)