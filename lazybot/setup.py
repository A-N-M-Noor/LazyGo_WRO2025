from setuptools import find_packages, setup

package_name = 'lazybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            # 'camera_node = lazybot.camera:main',
            # 'camera_test = lazybot.cam_test:main',
            # 'server_node = lazybot.server:main',
            'control = lazybot.control:main',
            'serial = lazybot.ser:main',
        ],
    },
)
