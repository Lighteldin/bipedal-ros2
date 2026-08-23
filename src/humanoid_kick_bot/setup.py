import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'humanoid_kick_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='you@example.com',
    description='Autonomous ball-kicking humanoid lower body: perception, FK/IK planning, servo control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_detector_node = humanoid_kick_bot.ball_detector_node:main',
            'ik_planner_node = humanoid_kick_bot.ik_planner_node:main',
            'servo_controller_node = humanoid_kick_bot.servo_controller_node:main',
        ],
    },
)
