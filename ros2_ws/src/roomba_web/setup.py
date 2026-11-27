from setuptools import find_packages, setup

package_name = 'roomba_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Keklak',
    maintainer_email='noreply@example.com',
    description='Websocket bridge between ROS and the Roomba web UI.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_bridge = roomba_web.bridge:main',
        ],
    },
)
