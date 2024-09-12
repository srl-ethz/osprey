from setuptools import setup
import os
from glob import glob

package_name = 'spectacular_vio'

setup(
    name=package_name,
    version='0.9.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soft Robotics Lab',
    maintainer_email='rkk@ethz.ch',
    description='Visual Inertial Odometry using Spectacular AI',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spectacular_node = spectacular_vio.spectacular_node:main',
            'vicon_object_publisher = spectacular_vio.vicon_obj_publisher:main',
        ],
    },
)
