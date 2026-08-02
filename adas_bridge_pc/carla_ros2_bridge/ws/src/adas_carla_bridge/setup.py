from setuptools import find_packages, setup

package_name = 'adas_carla_bridge'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='adas',
    maintainer_email='dev@adas.local',
    description='CARLA ↔ ADAS SoC 栈 ROS2 桥（IOT_TI HIL 闭环 PC 端）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = adas_carla_bridge.bridge_node:main',
        ],
    },
)
