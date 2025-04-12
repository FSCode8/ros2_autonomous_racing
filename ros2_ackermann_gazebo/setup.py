from setuptools import setup
import os
from glob import glob

package_name = 'ros2_ackermann_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',['urdf/test_ackermann_drive.xacro.urdf']),
        ('share/' + package_name + '/world',glob(os.path.join('world', '*.sdf'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sensorslab',
    maintainer_email='sensorslab@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
