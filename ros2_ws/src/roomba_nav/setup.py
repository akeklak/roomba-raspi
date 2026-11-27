from glob import glob

from setuptools import find_packages, setup

package_name = 'roomba_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Keklak',
    maintainer_email='noreply@example.com',
    description='Mapping, localization and navigation for the Roomba (slam_toolbox, AMCL, Nav2).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_manager = roomba_nav.map_manager:main',
            'nav_test = roomba_nav.nav_test:main',
        ],
    },
)
