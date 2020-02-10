# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from setuptools import setup

package_name = 'free_fleet_test_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
          ]
        )
    ],
    install_requires=['setuptools'],
    author='Aaron',
    author_email='aaron@openrobotics.org',
    zip_safe=True,
    maintainer='Aaron',
    maintainer_email='aaron@openrobotics.org',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package of testing scripts for free fleet ROS2 components',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
           'send_mode_request = free_fleet_test_ros2.send_mode_request:main',
           'send_path_request = free_fleet_test_ros2.send_path_request:main',
           'send_destination_request = free_fleet_test_ros2.send_destination_request:main'
       ],
    },
)