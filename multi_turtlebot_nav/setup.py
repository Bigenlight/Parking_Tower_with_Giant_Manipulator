from setuptools import setup
import os
from glob import glob

package_name = 'multi_turtlebot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 필요시 launch 파일 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Multi TurtleBot navigation using Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_sender = multi_turtlebot_nav.goal_sender:main',
            'turtlebot_state_monitor = multi_turtlebot_nav.turtlebot_state_monitor:main'
        ],
    },
)
