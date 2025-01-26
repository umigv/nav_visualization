from setuptools import setup
import os

package_name = 'nav_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/demo.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/costmap.txt']),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Navigation Visualization Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'visualizer_node = nav_visualization.visualizer_node:main',
            'dummy_publisher = nav_visualization.dummy_publisher:main',
        ],
    },
)