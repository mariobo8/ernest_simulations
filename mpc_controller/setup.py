import os
from glob import glob
from setuptools import setup

package_name = 'mpc_controller'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='joe s package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'mpc_node_position = mpc_controller.mpc_node_position:main',
            #'mpc_node_demo = mpc_controller.mpc_node_carexample:main',
            'utils = mpc_controller.utils',
            #'ackermann_solve = mpc_controller.ackermann_kinematics',
            #'pivot_solve = mpc_controller.pivot_4ws_kinematics',
            'mpc_node = mpc_controller.mpc_node:main',
            'stop_motors = mpc_controller.stop_motors:main',
            'test = mpc_controller.test_actuators:main',
            'pid_node = mpc_controller.pid_node:main',
            'sensors_node = mpc_controller.sensors_node:main'
        ],
    },
)