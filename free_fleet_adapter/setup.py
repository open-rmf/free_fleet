from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'free_fleet_adapter'

setup(
    name=package_name,
    version='2.4.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaron Chong',
    maintainer_email='aaronchong@intrinsic.ai',
    description='Free Fleet adapter for interfacing with Open-RMF core libraries',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=free_fleet_adapter.fleet_adapter:main',
        ],
    },
)
