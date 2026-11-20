from setuptools import find_packages, setup

package_name = 'forklift_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='vishwa',
    maintainer_email='vishwa@todo.todo',
    description='ROS 2 cmd_vel to ESP32 motor velocity bridge',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_to_esp32 = forklift_bridge.cmd_vel_to_esp32:main',
        ],
    },
)
