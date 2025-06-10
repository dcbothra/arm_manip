#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'ur_moveit_config'
    pkg_share = get_package_share_directory(pkg_name)
    ur_name = 'ur_description'
    ur_path = get_package_share_directory(ur_name)

    # URDF via xacro
    urdf_file = os.path.join(ur_path, 'build', 'ur.urdf')
    robot_description = {'robot_description':
        Command([FindExecutable(name='xacro'), ' ', urdf_file])}

    # 1) robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # # 2) Ignition Gazebo (empty world)
    # ign_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('ur_simulation'),
    #             'launch',
    #             'ign_gazebo.launch.py'
    #         )
    #     ),
    #     launch_arguments={'ign_args': '-r empty.sdf'}.items()
    # )

    # 3) ros2_control_node (loads controllers)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(pkg_share, 'config', 'ros2_controllers.yaml'),
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 4) Controller spawners (delay slightly to let controller_manager start)
    spawn_joint_state = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )
    spawn_manipulator = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['manipulator_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    # 5) Spawn robot into Ignition (from the robot_description topic)
    spawn_entity = Node(
        package='ur_simulation',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ur10e',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # 6) MoveIt2 demo (plans & executes using the controllers above)
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'demo.launch.py')
        ),
        # ensure it uses sim time and robot_description already up
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        # URDF → robot_description
        rsp_node,

        # Ignition simulation
        # ign_launch,

        # Load ros2_control_node
        ros2_control_node,

        # Controller spawners
        spawn_joint_state,
        spawn_manipulator,

        # Spawn into Ignition
        spawn_entity,

        # Finally, bring up MoveIt2
        moveit_demo,
    ])
