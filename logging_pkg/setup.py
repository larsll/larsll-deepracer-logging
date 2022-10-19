from setuptools import setup

package_name = 'logging_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lars Lorentz Ludvigsen',
    maintainer_email='larsll@outlook.com',
    description='ROS2 package that enables creation of rosbags for DeepRacer',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_log_node = logging_pkg.bag_log_node:main'
        ],
    },
)
