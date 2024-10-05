from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_fsm_bumpgo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), (glob("launch/*.launch.py")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drint',
    maintainer_email='drint@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bump_go_main = robot_fsm_bumpgo_py.bump_go_main:main"
        ],
    },
)
