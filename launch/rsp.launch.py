import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    pkg_name = 'control_1'
    pkg_path = get_package_share_directory('control_1')

    xacro_file = os.path.join(pkg_path, 'description', 'kasytwin.urdf.xacro')

    # Get URDF via xacro
    robot_description_content = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(pkg_name),
            'bringup',
            'config',
            'config.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Static transform: world -> odom
    node_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    # Convert odometry to TF (odom -> base_link)
    odom_to_tf_node = Node(
        package='control_1',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pid_controller_spawner = Node(
        package='control_1',
        executable='pid_control_node',
        name='pid_controller',
        output='screen',
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
    )

    reference_manager_spawner = Node(
        package='control_1',
        executable='reference_manager_node',
        name='reference_manager',
        output='screen',
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
    )

    motor_command_spawner = Node(
        package='control_1',
        executable='motor_command_node',
        name='motor_command',
        output='screen',
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
    )

    imu_quat_to_euler_spawner = Node(
        package='control_1',
        executable='imu_quat_to_euler_node',
        name='imu_quat_to_euler',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    laserscan_to_pointcloud_spawner = Node(
        package='control_1',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pca_node_spawner = Node(
        package='control_1',
        executable='pca_node',
        name='pca_node',
        output='screen',
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
    )

    # Health monitor
    health_monitor_process = ExecuteProcess(
        cmd=['ros2', 'run', 'control_1', 'topic_health_monitor_node', '--ros-args', '--params-file', robot_controllers],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='If true, use simulated clock'),
        node_robot_state_publisher,
        node_world_to_odom,
        odom_to_tf_node,
        pid_controller_spawner,
        reference_manager_spawner,
        motor_command_spawner,
        imu_quat_to_euler_spawner,
        laserscan_to_pointcloud_spawner,
        pca_node_spawner,
        health_monitor_process,
    ])