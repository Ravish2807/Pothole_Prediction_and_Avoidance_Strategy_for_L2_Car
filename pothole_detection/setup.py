from setuptools import find_packages, setup

package_name = 'pothole_detection'

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
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	'pothole_detection_node = pothole_detection.yolo_pothole_detection:main',
    'pothole_mapper_node = pothole_detection.yolo_pothole_depth:main',
    'unity_pothole_detection_node = pothole_detection.unity_pothole_detection_node:main', 
        ],
    },
)
