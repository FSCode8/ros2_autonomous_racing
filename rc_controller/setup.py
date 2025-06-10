from setuptools import find_packages, setup

package_name = 'rc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vehicleint',
    maintainer_email='vehicleint@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_controller = rc_controller.drive_10_meter:main',
            'edge_detection = rc_controller.edge_detection:main',
            'get_images = rc_controller.get_images:main',
            'camera_calibration = rc_controller.camera_calibration:main',
            'lane_detection = rc_controller.lane_detection:main',
            'lane_detection_2 = rc_controller.lane_detection_2:main',
            'black_and_white_img = rc_controller.black_and_white_img:main',
            'drive_auto = rc_controller.drive_auto:main',
            'birdseyeview = rc_controller.birdseyeview:main',
            'traffic_light_detection = rc_controller.traffic_light_detection:main',
            'color_detection = rc_controller.color_detection:main'
        ],
    },
)
