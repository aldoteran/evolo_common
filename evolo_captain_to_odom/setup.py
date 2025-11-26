from setuptools import find_packages, setup

package_name = 'evolo_captain_to_odom'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benjamin',
    maintainer_email='nrol@kth.se',
    description='',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'captain_odom = evolo_captain_to_odom.captain_to_odom:main',
            'odom_initializer = evolo_captain_to_odom.captain_map_odom_initializer:main',
        ],
    },
)
