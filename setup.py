from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

d = generate_distutils_setup(
    packages=['ros_slam_webui'],
    package_dir={'': '.'}
)
setup(**d)
