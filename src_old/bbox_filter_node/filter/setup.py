from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='greta',
    maintainer_email='greta@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = filter.bbox_filter_node:main',
            'listener = filter.viewer_node:main',
            'listener_flip = filter.viewer_node_flip:main',
            'color = filter.fake_color_string:main',
            'shoot = filter.shoot_prediction_without_drag:main',
            'shoot_flip = filter.shoot_prediction_without_drag_flip:main',
            'depth = filter.depth_node:main',
            'imu_filter = filter.imu_filter_node:main',
            'bbox_shoot = filter.bbox_filter_node_plus_shooting:main',
        ],
    },
)
