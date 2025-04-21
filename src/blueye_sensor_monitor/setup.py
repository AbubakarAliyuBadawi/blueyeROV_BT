from setuptools import setup
import os
from glob import glob

package_name = 'blueye_sensor_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROV sensor monitoring and testing tools',
    license='Apache License 2.0',
    # Replace tests_require with this:
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sensor_visualizer_node = blueye_sensor_monitor.sensor_visualizer_node:main',
            'sensor_failure_simulator = blueye_sensor_monitor.sensor_failure_simulator:main',
            'battery_monitor_node = blueye_sensor_monitor.battery_monitor_node:main',
        ],
    },
)