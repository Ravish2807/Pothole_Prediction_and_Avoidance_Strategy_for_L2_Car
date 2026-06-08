from setuptools import find_packages, setup

package_name = 'obstacle_detection'

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
    maintainer='ravish',
    maintainer_email='ravish@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolvo_obstacle_detection = obstacle_detection.yolvo_obstacle_detection:main',
            'safety_fusion_node = obstacle_detection.safety_fusion_node:main',
            'path_guard_node = obstacle_detection.path_guard_node:main',
            'lane_guide_node = lane_detection.lane_guide_node:main',
            'safety_mux_node = obstacle_detection.safety_mux_node:main',
            'unity_yolo_obstacle_node = obstacle_detection.unity_yolo_obstacle_node:main',
        ],
    },
)
