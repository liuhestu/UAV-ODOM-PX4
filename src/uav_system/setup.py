from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
# Domain packages use Python 3 namespace packages; catkin generates devel relays.
setup(**generate_distutils_setup(
    packages=['uav_core', 'mission_executor', 'flight_supervisor', 'control_backend'],
    package_dir={'': 'src', 'uav_core': 'src/support/uav_core'}))
