from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/read_write.py'],
    packages=['humanoid_robot_intelligence_control_system_read_write_demo'],
    package_dir={'': 'src'},
)

setup(**setup_args)
