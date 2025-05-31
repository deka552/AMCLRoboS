from setuptools import setup
import os
from glob import glob

package_name = 'pointcloud_to_laserscan'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Convert 3D PointCloud2 to 2D LaserScan for AMCL localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_to_laserscan_node = pointcloud_to_laserscan.pointcloud_to_laserscan_node:main',
        ],
    },
)
