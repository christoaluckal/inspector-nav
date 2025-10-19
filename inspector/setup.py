from setuptools import find_packages, setup
import glob
import os

package_name = 'inspector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/clearpath_standard_demo.launch.py',
            'launch/custom_nav2_launch.py',
            'launch/mola_husky_slam.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/husky.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azaher',
    maintainer_email='azaher@todo.todo',
    description='Autonomous inspection robot using Clearpath Husky with slam_toolbox',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspector = inspector.inspector:main',
        ],
        'launch.launch_description': [
            'navigation = inspector.launch.navigation:generate_launch_description',
            'clearpath_standard_demo = inspector.launch.clearpath_standard_demo:generate_launch_description',
            'mola_husky_slam = inspector.launch.mola_husky_slam:generate_launch_description',
        ],
    },
)
