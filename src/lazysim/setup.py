from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'lazysim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.py')),
        (os.path.join('share', package_name, "description"), glob('description/*.xacro')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, "worlds"), glob('worlds/*.world')),
        (os.path.join('share', package_name, "worlds","materials","scripts"), glob('worlds/materials/scripts/*')),
        (os.path.join('share', package_name, "worlds","materials","textures"), glob('worlds/materials/textures/*')),
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
            'lazybridge = lazysim.lazybridge:main',
            'track_maker = lazysim.track_maker:main',
        ],
    },
)
