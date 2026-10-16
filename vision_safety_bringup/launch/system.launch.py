"""The whole vision safety system with one command.

  ros2 launch vision_safety_bringup system.launch.py
  ros2 launch vision_safety_bringup system.launch.py cruise:=false          # drive yourself
  ros2 launch vision_safety_bringup system.launch.py headless:=true rviz:=false

Nodes:
  Gazebo (corridor world) + ros_gz_bridge + robot_state_publisher   [vision_safety_sim]
  obstacle_detector   camera -> perception/obstacle (+ debug overlay)
  safety_monitor      perception/obstacle + cmd_vel_raw -> cmd_vel   (the ONLY cmd_vel publisher)
  cruise_driver       obstacle-blind constant request on cmd_vel_raw (optional)
  rviz2               robot, TF, odometry trail, debug overlay       (optional)

To drive manually, ALWAYS remap teleop to cmd_vel_raw:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('vision_safety_bringup')
    sim_share = get_package_share_directory('vision_safety_sim')
    config = os.path.join(bringup_share, 'config')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    debug_view = LaunchConfiguration('debug_view')
    cruise = LaunchConfiguration('cruise')
    cruise_speed = LaunchConfiguration('cruise_speed')
    log_level = LaunchConfiguration('log_level')

    args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'corridor.sdf'),
            description='Gazebo world (SDF)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo server only (no GUI)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('debug_view', default_value='false',
                              description='Extra rqt_image_view window on the debug overlay'),
        DeclareLaunchArgument('cruise', default_value='true',
                              description='Start cruise_driver (constant forward request)'),
        DeclareLaunchArgument('cruise_speed', default_value='0.3',
                              description='cruise_driver speed [m/s]'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level of the vision safety nodes'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world, 'headless': headless}.items(),
    )

    ros_args = ['--log-level', log_level]

    obstacle_detector = Node(
        package='vision_safety_ros', executable='obstacle_detector', name='obstacle_detector',
        output='screen', ros_arguments=ros_args,
        parameters=[os.path.join(config, 'obstacle_detector.yaml'), {'use_sim_time': True}],
    )

    safety_monitor = Node(
        package='vision_safety_ros', executable='safety_monitor', name='safety_monitor',
        output='screen', ros_arguments=ros_args,
        parameters=[os.path.join(config, 'safety_monitor.yaml'), {'use_sim_time': True}],
    )

    cruise_driver = Node(
        package='vision_safety_ros', executable='cruise_driver', name='cruise_driver',
        output='screen', ros_arguments=ros_args,
        parameters=[{'use_sim_time': True,
                     'speed': ParameterValue(cruise_speed, value_type=float)}],
        condition=IfCondition(cruise),
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(config, 'vision_safety.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    debug_viewer = Node(
        package='rqt_image_view', executable='rqt_image_view', name='debug_view',
        arguments=['/perception/debug_image'],
        condition=IfCondition(debug_view),
    )

    return LaunchDescription(args + [
        sim, obstacle_detector, safety_monitor, cruise_driver, rviz_node, debug_viewer,
    ])
