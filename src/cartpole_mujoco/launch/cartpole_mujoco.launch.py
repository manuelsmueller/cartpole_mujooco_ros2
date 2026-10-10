"""Startet Simulation und PID-Regler.

Beispiele:
  ros2 launch cartpole_mujoco cartpole_mujoco.launch.py                 # Primitiv-Modell
  ros2 launch cartpole_mujoco cartpole_mujoco.launch.py model:=mesh     # Mesh-Modell
  ros2 launch cartpole_mujoco cartpole_mujoco.launch.py viewer:=false   # headless
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context).lower() in ('true', '1', 'yes')

    share = get_package_share_directory('cartpole_mujoco')
    model_file = os.path.join(share, 'model', f'cartpole_{model}.xml')
    params_file = os.path.join(share, 'config', f'pid_{model}.yaml')
    for path in (model_file, params_file):
        if not os.path.isfile(path):
            raise RuntimeError(f"Datei fehlt: {path} (gueltige Werte fuer model: primitive, mesh)")

    return [
        Node(
            package='cartpole_mujoco',
            executable='simulation_node',
            output='screen',
            parameters=[{'model_file': model_file, 'viewer': viewer}],
        ),
        Node(
            package='cartpole_mujoco',
            executable='pid_controller_node',
            output='screen',
            parameters=[params_file],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='primitive',
                              description="'primitive' oder 'mesh'"),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='MuJoCo-Viewer oeffnen'),
        OpaqueFunction(function=_setup),
    ])
