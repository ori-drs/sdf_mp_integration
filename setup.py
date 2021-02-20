## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    scripts=['scripts/moving_obstacle_node.py'],
    packages=['sdf_mp_integration'],
    package_dir={'': 'src'})

setup(**setup_args)