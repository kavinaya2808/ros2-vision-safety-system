"""Launch Gazebo Harmonic with the corridor world, spawn vision_bot, and bridge topics.

Usage:
  ros2 launch vision_safety_sim sim.launch.py               # with Gazebo GUI
  ros2 launch vision_safety_sim sim.launch.py headless:=true  # server only (tests / CI)
  ros2 launch vision_safety_sim sim.launch.py world:=/abs/path/other.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_safety_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'corridor.sdf'),
            description='Absolute path to the Gazebo world (SDF)'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server only, with headless rendering'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
    ]

    gz_launch = PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])

    # -r: start running immediately. -v 3: warnings + info.
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -v 3 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless),
    )

    # -s: server only. --headless-rendering: camera still renders, via EGL, without a display.
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-s -r -v 3 --headless-rendering ', world],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless),
    )

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'vision_bot.urdf.xacro')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_vision_bot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'vision_bot',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.01',
            '-Y', spawn_yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription(declare_args + [
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
