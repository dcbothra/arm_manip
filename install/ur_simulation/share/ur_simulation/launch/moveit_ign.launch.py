from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directories
    ur_description_dir = get_package_share_directory('ur_description')
    moveit_config_dir = get_package_share_directory('ur_moveit_config')
    ur_simulation_dir = get_package_share_directory('ur_simulation')

    # Paths
    robot_description_path = os.path.join(ur_description_dir, 'build', 'ur.urdf')
    config_dir = os.path.join(moveit_config_dir, 'config')
    world_path = os.path.join(ur_simulation_dir, 'worlds', 'empty.sdf')

    # Read URDF as string for MoveIt and controller_manager
    robot_description = {
        "robot_description": open(robot_description_path).read()
    }

    # OMPL planning pipeline
    ompl_pipeline_config = os.path.join(config_dir, "planning_pipelines.yaml")

    return LaunchDescription([
        # 1. Launch Ignition Gazebo with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_ign_gazebo'),
                    'launch/ign_gazebo.launch.py')
            ]),
            launch_arguments={
                'world': world_path,
                'ign_args': '-v 4'
            }.items()
        ),

        # 2. Spawn robot in Ignition
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-name', 'ur10e',
                '-file', robot_description_path,
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),

        # 3. Launch ros2_control with robot description and controller config
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': open(robot_description_path).read()},
                os.path.join(moveit_config_dir, 'config', 'ros2_controllers.yaml')
            ],
            output='screen'
        ),

        # 4. Spawn controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen'
        ),

        # 5. Launch MoveIt (make sure it loads pipelines internally)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')
            ])
        ),
    ])
