from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'human_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('params', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayooran',
    maintainer_email='mayoo4234@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_merge_node = human_robot_pkg.merge_map:main',
            'frontier_pub = human_robot_pkg.frontier:main',
            'simple_navigator = human_robot_pkg.simple_navigator:main',
            'param_loader = human_robot_pkg.param_loader:main',
            'map_logger = human_robot_pkg.map_logger:main',
            'odom_publisher = human_robot_pkg.odom_publisher:main',
            'image_visualiser = human_robot_pkg.image_subscriber:main',
            'point_to_nav_goal = human_robot_pkg.point_to_nav_goal:main',
            'scan_limiter = human_robot_pkg.scan_limiter:main',
            'frontier_navigator = human_robot_pkg.frontier_navigator:main'
        ],
    },
)
