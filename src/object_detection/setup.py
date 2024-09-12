from setuptools import setup
import os
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.9.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soft Robotics Lab',
    maintainer_email='rkk@ethz.ch',
    description='Object Detection using Detectron',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_sender_node = object_detection.video_sender_node:main',
            'video_receiver_node = object_detection.video_receiver_node:main',
            'object_detector_node = object_detection.detection:main',
            'webcam_publisher = object_detection.webcam_publisher:main',
        ],
    },
)
