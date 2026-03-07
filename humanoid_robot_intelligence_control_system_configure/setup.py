# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys

# Determine ROS version from environment variable
ros_version = os.getenv('ROS_VERSION', '1')  # Default to ROS 1 if not set

# Handle ROS 1 (uses distutils and catkin_pkg)
if ros_version == '1':
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    # Fetch values from package.xml for ROS 1
    setup_args = generate_distutils_setup(
        scripts=[
            'src/face_follower.py',
            'src/face_tracker.py',
            'src/object_follower.py',
            'src/object_tracker.py'
        ],
        packages=['humanoid_robot_intelligence_control_system_configure'],
        package_dir={'': 'src'},
    )

    setup(**setup_args)

# Handle ROS 2 (uses setuptools)
elif ros_version == '2':
    from setuptools import setup, find_packages

    package_dir = 'src'
    packages = find_packages(where=package_dir)

    setup(
        name='humanoid_robot_intelligence_control_system_configure',
        version='0.1.0',
        packages=packages,
        package_dir={'': package_dir},
        scripts=[
            'src/face_follower.py',
            'src/face_tracker.py',
            'src/object_follower.py',
            'src/object_tracker.py'
        ],
        install_requires=[
            'setuptools',
            # Add other dependencies as needed
        ],
        zip_safe=True,
        description="AI system for humanoid robot intelligence configure",
        license='GPLv3',
        author='Ronaldson Bellande',
        author_email='ronaldsonbellande@gmail.com',
        url='https://github.com/Robotics-Sensors/humanoid_robot_intelligence_control_system_architecture'
    )

else:
    sys.stderr.write(f"Unsupported ROS_VERSION: {ros_version}. Please set ROS_VERSION to '1' or '2'.\n")
    sys.exit(1)   
