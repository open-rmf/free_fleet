from setuptools import setup

package_name = 'free_fleet_server_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=['free_fleet_server_py'],
    py_modules=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    zip_safe=True,
    maintainer='Morgan Quigley',
    maintainer_email='morgan@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=package_name,
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'free_fleet_server_node=free_fleet_server_py.free_fleet_server_py:main',
        ],
    },
)
