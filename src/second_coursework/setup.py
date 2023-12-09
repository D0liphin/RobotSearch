from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup # type: ignore

setup(**generate_distutils_setup(
    packages=['util', 'nodes', 'states'],
    package_dir={'': 'src'}
))