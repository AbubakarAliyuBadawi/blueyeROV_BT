from setuptools import setup
import os
from glob import glob

package_name = 'gz_battery_management'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Battery management and dock distance calculation nodes',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest', 'mock']
    },
    entry_points={
        'console_scripts': [
            'dock_distance_calc = gz_battery_management.dock_distance_calc:main',
            'battery_management = gz_battery_management.battery_management:main'
        ],
    },
)