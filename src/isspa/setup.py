from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['isspa'],
    package_dir={'': 'scripts'},
    version='0.1.0'
)

setup(**d)