"""Setup script for nav_monitor package."""

from setuptools import find_packages, setup

package_name = 'nav_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taneesh',
    maintainer_email='taneesh.sawant0110@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'monitor_node = nav_monitor.monitor:main',
            'aruco_detector = nav_monitor.aruco_detector:main',
        ],
    },
)
