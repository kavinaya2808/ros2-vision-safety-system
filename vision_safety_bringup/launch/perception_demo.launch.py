"""Phase 3 demo: simulation + obstacle_detector + live debug view.

  ros2 launch vision_safety_bringup perception_demo.launch.py
  ros2 launch vision_safety_bringup perception_demo.launch.py debug_view:=false
  ros2 launch vision_safety_bringup perception_demo.launch.py headless:=true

Drive with teleop in another terminal (publishes /cmd_vel directly in this phase):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('vision_safety_bringup')
    sim_share = get_package_share_directory('vision_safety_sim')

    headless = LaunchConfiguration('headless')
    debug_view = LaunchConfiguration('debug_view')
    params_file = LaunchConfiguration('params_file')

    args = [
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without GUI'),
        DeclareLaunchArgument('debug_view', default_value='true',
                              description='Open rqt_image_view on the debug overlay'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'config', 'obstacle_detector.yaml'),
            description='obstacle_detector parameter file'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    obstacle_detector = Node(
        package='vision_safety_ros',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    debug_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='debug_view',
        arguments=['/perception/debug_image'],
        condition=IfCondition(debug_view),
    )

    return LaunchDescription(args + [sim, obstacle_detector, debug_viewer])
