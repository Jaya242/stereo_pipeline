import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stereo_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files in installation directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ctrlescflyyy',
    maintainer_email='ctrlescflyyy@todo.todo',
    description='ROS 2 package for RTSP stereo camera streaming, multi-color object detection, and obstacle control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_camera_node = stereo_pipeline.rtsp_stream:main',
            'stereo_processor = stereo_pipeline.stereo_processor:main',
            'obstacle_controller = stereo_pipeline.obstacle_controller:main',
            'sensor_talker = stereo_pipeline.sensor_talker:main',
            'car_listener = stereo_pipeline.car_listener:main',
        ],
    },
)
