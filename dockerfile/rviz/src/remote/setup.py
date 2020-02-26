# Copyright (c) 2019 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/micro-ROS/micro-ROS_kobuki_demo.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages
from setuptools import setup

package_name = 'micro-ros_crazyflie_demo_remote'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', ['config/crazyflie.rviz']),
        ('share/' + package_name + '/config/', ['config/crazyflie_attitude.rviz']),
        ('share/' + package_name + '/launch/', ['launch/launch_drone_attitude.launch.py']),
        ('share/' + package_name + '/launch/', ['launch/launch_drone_position.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Pablo Garrido',
    author_email='pablogarrido@eprosima.com',
    maintainer='Pablo Garrido',
    maintainer_email='pablogarrido@eprosima.com',
    keywords=['micro-ROS', 'Crazyflie', 'demo'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'attitude_to_vel = attitude_to_vel.attitude_to_vel:main',
            'simple_navigation = simple_navigation.simple_navigation:main',
        ],
    },
)
