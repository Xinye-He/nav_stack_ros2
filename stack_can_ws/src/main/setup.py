from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'main'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]

setup(
    name=package_name,
    version='0.3.1',
    packages=find_packages(),          # 扁平布局：直接查找当前包
    package_dir={'': '.'},             # 根就是包根
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xinye',
    maintainer_email='you@example.com',
    description='Waypoint PID follower with VCU CAN control (pre-speed discrete & 3-level angle), RTK heading, action override.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traj_waypoint_follower = main.traj_waypoint_follower:main',
            'stack_can_executor     = main.stack_can_executor:main',
            'teleop_key             = main.teleop_key:main',
            'rtk_center_from_nmea   = main.rtk_center_from_nmea:main',
            'dr_odometry_node       = main.dr_odometry_node:main',
            'websocket_teleop_key   = main.websocket_teleop_key:main',
            'bale_align_controller  = main.bale_align_controller:main',
            'geofence_monitor       = main.geofence_monitor:main',
        ],
    },
)
