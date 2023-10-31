from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2rrt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='daviddorf2023@u.northwestern.edu',
    description='RRT based exploration in both 2D and 3D',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt2D = ros2rrt.rrt2D:main',
            'rrt3D = ros2rrt.rrt3D:main',
            'map_frame_pub = ros2rrt.map_frame_pub:main',
            'occupancy_pub = ros2rrt.occupancy_pub:main',
        ],
    },
)
