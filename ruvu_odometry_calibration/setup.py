from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ruvu_odometry_calibration'],
    package_dir={'': 'src'}
)

setup(**d)
