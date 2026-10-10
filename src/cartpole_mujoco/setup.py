import os
from glob import glob

from setuptools import setup

package_name = 'cartpole_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'model'), glob('cartpole_mujoco/model/*.xml')),
        (os.path.join('share', package_name, 'model', 'meshes'),
         glob('cartpole_mujoco/model/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='manu@example.com',
    description='Cart Pole in MuJoCo mit ROS2-Anbindung',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulation_node = cartpole_mujoco.simulation_node:main',
            'pid_controller_node = cartpole_mujoco.pid_controller_node:main',
        ],
    },
)
