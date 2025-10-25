import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'xbox_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),           
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cg',
    maintainer_email='natthawejumjai@gmail.com',
    description='Xbox Monitor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'xbox_monitor = xbox_monitor.xbox_monitor:main',
        ],
    },
)
