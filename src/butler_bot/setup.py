from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'butler_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name), glob('launch/*.launch.py')),
                (os.path.join('share', package_name), glob('config/*.yaml')),
                (os.path.join('share', package_name), glob('map/*')),
                (os.path.join('share', package_name), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e-yanthra',
    maintainer_email='aftaabahamed2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_manager = butler_bot.robot_manager:main',
            'conf_server = butler_bot.conf_server:main',
        ],
    },
)
