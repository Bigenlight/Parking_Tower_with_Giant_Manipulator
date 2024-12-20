from setuptools import setup
import os
from glob import glob

package_name = 'robot_control_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 필요하다면 지도 이미지나 YAML 파일을 추가 가능
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools', 'pyqt5'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='PyQt5 GUI for ROS2 TurtleBot Control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_gui = robot_control_gui.robot_control_gui:main',
            'robot_control_gui2 = robot_control_gui.robot_control_gui2:main',
            'monitoring = robot_control_gui.monitoring:main',
            
        ],
    },
)
