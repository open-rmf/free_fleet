from setuptools import setup

package_name = 'free_fleet_server_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=['free_fleet_server_test'],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Aaron Chong',
    author_email='aaron@osrfoundation.org',
    zip_safe=False,
    maintainer='Aaron Chong',
    maintainer_email='aaron@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='package for testing free_fleet_server',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    scripts=[
        'free_fleet_server_test/send_mode_request.py',
        'free_fleet_server_test/send_destination_request.py',
        'free_fleet_server_test/send_path_request.py'],
    entry_points={},
)
