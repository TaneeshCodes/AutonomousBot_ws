"""Setup script for assessment_launch package."""

from setuptools import find_packages, setup

package_name = 'assessment_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/assessment.launch.py']),
        ('share/' + package_name + '/param', ['param/burger.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/supermarket_arena.sdf']),
        ('share/' + package_name + '/textures', [
            'textures/marker_0.png', 'textures/marker_1.png', 'textures/marker_2.png',
            'textures/marker_3.png', 'textures/marker_4.png', 'textures/marker_5.png',
            'textures/marker_6.png', 'textures/marker_7.png', 'textures/marker_8.png',
            'textures/marker_9.png', 'textures/marker_10.png', 'textures/marker_11.png',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/map.yaml',
            'maps/map.pgm',
            'maps/supermarket_mapped.yaml',
            'maps/supermarket_mapped.pgm'
        ]),
        ('lib/' + package_name, ['scripts/patrol_waypoints.py']),
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
        ],
    },
)
