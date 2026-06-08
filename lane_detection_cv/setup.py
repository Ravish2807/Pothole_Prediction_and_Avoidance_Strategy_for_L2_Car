from setuptools import find_packages, setup

package_name = 'lane_detection_cv'

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
		'lane_cv_node = lane_detection_cv.lane_detection_cv:main',
        'unity_lane_detection_node = lane_detection_cv.unity_lane_detection_node:main',
        ],
    },
)
