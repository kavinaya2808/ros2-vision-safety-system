"""Phase 4 demo, kept for convenience: system.launch.py with the rqt debug window
instead of RViz. Accepts the same arguments as system.launch.py.

  ros2 launch vision_safety_bringup safety_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('vision_safety_bringup')
    passthrough = ['headless', 'cruise', 'cruise_speed']
    defaults = {'headless': 'false', 'cruise': 'true', 'cruise_speed': '0.3'}

    args = [DeclareLaunchArgument(name, default_value=defaults[name]) for name in passthrough]
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'system.launch.py')),
        launch_arguments=dict(
            {name: LaunchConfiguration(name) for name in passthrough},
            rviz='false', debug_view='true').items(),
    )
    return LaunchDescription(args + [system])
