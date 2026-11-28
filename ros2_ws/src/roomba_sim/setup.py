from glob import glob

from setuptools import find_packages, setup

package_name = 'roomba_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Keklak',
    maintainer_email='noreply@example.com',
    description='Gazebo simulation of the Roomba 675 + Raspberry Pi robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'smoke_test = roomba_sim.smoke_test:main',
            'world_markers = roomba_sim.world_markers:main',
            'ground_truth_localizer = roomba_sim.ground_truth_localizer:main',
            'realtime_pacer = roomba_sim.realtime_pacer:main',
        ],
    },
)
