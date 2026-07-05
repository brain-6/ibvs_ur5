import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ibvs_ur5')

    # ── 自定义世界文件路径 ──
    world_file = os.path.join(pkg_share, 'worlds', 'visual_servo.sdf')

    # ── 读取预生成的 URDF ──
    urdf_path = '/tmp/ur5_gazebo.urdf'
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_desc_param = {'robot_description': robot_description}

    # ── 启动 Gazebo，加载自定义世界 ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # ── robot_state_publisher ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_desc_param, {'use_sim_time': True}],
        output='screen'
    )

    # ── 在 Gazebo 中生成 UR5 ──
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur5', '-topic', '/robot_description', '-z', '0.1'],
        output='screen'
    )

    # ── ros_gz_bridge：桥接相机图像 ──
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # ── 控制器延迟启动 ──
    joint_state_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    traj_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        joint_state_spawner,
        traj_spawner
    ])
