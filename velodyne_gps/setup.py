import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'velodyne_gps'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        #(os.path.join('share', PACKAGE_NAME), glob('launch/*.launch.py'))
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools',
                      'pyyaml'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for UDP data parsing and publishing GPRMC data',
    license='Apache License 2.0',
    keywords=['ROS2'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velodyne_gps = velodyne_gps.velodyne_gps_udp:main'
        ],
    },
)