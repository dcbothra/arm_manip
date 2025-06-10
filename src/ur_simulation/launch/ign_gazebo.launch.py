#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to packages
    ur_desc = get_package_share_directory('ur_description')
    ursim   = get_package_share_directory('ur_simulation')
    ign_pkg = get_package_share_directory('ros_ign_gazebo')

    # 1) Process the XACRO into a Robot Description parameter
    xacro_file = os.path.join(ur_desc, 'urdf', 'ur_mocked.urdf.xacro')
    robot_description = {
      'robot_description': Command([
          'xacro ', xacro_file,
          ' name:=ur10e',
          ' transmission_hw_interface:=hardware_interface/PositionJointInterface',
          " tf_prefix:=''"
      ])
    }

    # 2) Start robot_state_publisher so tf and robot_description get broadcast
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3) Launch Ignition Gazebo with your empty world
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ign_pkg, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={
            'ign_args': '-r ' + os.path.join(ursim, 'worlds', 'empty.sdf')
        }.items()
    )

    # 4) Spawn the robot into Ignition from the robot_description topic
    spawn_entity = Node(
      package='ros_ign_gazebo',
      executable='create',
      output='screen',
      arguments=[
        '-topic', 'robot_description',
        '-entity', 'ur10e'
      ]
    )

    # 5) Start a joint_state_broadcaster so ros2_control publishes joint states
    jsb_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster'],
      output='screen'
    )

    return LaunchDescription([
        ign_launch,
        rsp_node,
        spawn_entity,
        jsb_spawner,
    ])
