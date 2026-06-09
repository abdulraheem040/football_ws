from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'humanoid_football'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdul_raheem',
    maintainer_email='abdul_raheem@todo.todo',
    description='ROS 2 package for humanoid robots playing football in Gazebo simulation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller = humanoid_football.robot_controller:main',
            'ball_controller = humanoid_football.ball_controller:main',
        ],
    },
)
