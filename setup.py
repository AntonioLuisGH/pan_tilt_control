from setuptools import setup
import os
from glob import glob

package_name = 'pan_tilt_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- THIS IS THE CRITICAL SECTION YOU ARE MISSING ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # ----------------------------------------------------
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='antonio', # Changed to you
    maintainer_email='antonio@example.com', # Changed to you
    description='ROS 2 driver for DYNAMIXEL pan-tilt system via Arduino.',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # This makes your driver_node.py an executable
            'driver_node = pan_tilt_control.driver_node:main',
        ],
    },
)