__author__ = 'Marco Esposito'

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_easy_handeye'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy']
)

setup(**d)
