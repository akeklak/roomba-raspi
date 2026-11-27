from glob import glob

from setuptools import find_packages, setup

package_name = 'roomba_bringup'

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
        ('share/' + package_name + '/models/roomba_675',
         ['models/roomba_675/model.config', 'models/roomba_675/model.sdf']),
        ('share/' + package_name + '/models/roomba_675/meshes',
         glob('models/roomba_675/meshes/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Keklak',
    maintainer_email='noreply@example.com',
    description='Bringup for the Roomba 675 + Raspberry Pi 4 robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_guard = roomba_bringup.power_guard:main',
        ],
    },
)
