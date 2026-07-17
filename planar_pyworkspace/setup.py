from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'planar_pyworkspace'
submodules = 'planar_pyworkspace/submodules'
utils = 'planar_pyworkspace/utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))      

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='noora.sassali@gmail.com',
    description='Planar workspace package for defining a 2D workplane and running visualizations',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planar_pyworkspace_node = planar_pyworkspace.planar_pyworkspace_node:main',
            'aruco_pose_collector = planar_pyworkspace.aruco_pose_collector_node:main',
            'apriltag_pose_collector = planar_pyworkspace.apriltag_pose_collector_node:main',
            'visual_pose_collector = planar_pyworkspace.visual_pose_collector_node:main',
            'workspace_node = planar_pyworkspace.planar_workspace_node:main',
            'broadcaster_node = planar_pyworkspace.frame_broadcaster:main',
            'visualizer_node = planar_pyworkspace.visualizer_node:main',
        ],
    },
)
