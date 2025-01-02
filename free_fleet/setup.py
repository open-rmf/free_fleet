from setuptools import find_packages
from setuptools import setup

package_name = 'free_fleet'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaron Chong',
    maintainer_email='aaronchong@intrinsic.ai',
    description='A free fleet management library',
    license='Apache License 2.0',
    tests_require=['pytest'],
)
