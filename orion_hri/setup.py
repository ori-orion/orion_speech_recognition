from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['orion_hri'],
    scripts=['scripts/wait_for_instruction_server.py'],
    package_dir={'': 'src'}
)

setup(**d)
