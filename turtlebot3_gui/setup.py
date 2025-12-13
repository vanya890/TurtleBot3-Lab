#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['turtlebot3_gui'],
    package_dir={'': 'src'},
    scripts=[
        'src/turtlebot_gui.py',
        'src/camera_processor.py',
        'src/dual_camera_processor.py',
        'src/sensor_state_simulator.py'
    ],
)

setup(**d)
