from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['orion_hri'],
    scripts=['scripts/bring_me_server.py', 'scripts/wait_for_confirmation_server.py', 'scripts/wait_for_input_server.py'],
    package_dir={'': 'src'}
)

setup(**d)
