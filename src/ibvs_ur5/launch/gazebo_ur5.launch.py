import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = '/tmp/ur5_gazebo.urdf'
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_desc_param = {'robot_description': robot_description}

    # Gazebo via official launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_desc_param, {'use_sim_time': True}],
        output='screen'
    )

    # Spawn UR5
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur5', '-topic', '/robot_description', '-z', '0.1'],
        output='screen'
    )

    # Wait for Gazebo's internal controller_manager, then spawn controllers
    joint_state_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    traj_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner,
        traj_spawner
    ])
