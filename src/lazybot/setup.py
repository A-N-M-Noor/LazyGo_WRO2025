from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'lazybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duronto',
    maintainer_email='nafis.noor.202012@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = lazybot.control:main',
            'open_control = lazybot.open_control:main',
            'parking = lazybot.parking:main',
            'detect = lazybot.detection:main',
            'detect_cam = lazybot.detection_cam:main',
            'serial = lazybot.ser:main',
            'color_calibration_node = lazybot.color_calibration:main',
            'debug = lazybot.debug:main',
        ],
    },
)
