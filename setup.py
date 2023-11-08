import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_robot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nml',
    maintainer_email='nml@todo.todo',
    description='A ROS2 package for interfacing with the Mini and Desktop robot arms',
    license='Mozilla Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = ros_robot_arm.entry_point:main'
        ],
    },
)
