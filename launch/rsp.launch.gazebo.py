import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    pkg_name = 'control_1'
    pkg_path = os.path.join(
        get_package_share_directory('control_1'))

    xacro_file = os.path.join(pkg_path,
                              'description',
                              'kasytwin.urdf.xacro')

    # Get URDF via xacro
    robot_description_content  = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    # Get Tube model
    tube_sdf_file = os.path.join(pkg_path,
                                 'description',
                                 'tube_model.sdf')

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

    node_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]  
    )

    # Convert odometry to TF (odom -> base_link)
    odom_to_tf_node = Node(
        package='control_1',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'kasytwin', '-allow_renaming', 'true'],
    )

      # Spawn tube from SDF file directly
    gz_spawn_tube = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', tube_sdf_file,  
            '-name', 'tube',
            '-allow_renaming', 'true'
        ],
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

    # Health monitor - simple manual process without parameters
    health_monitor_process = ExecuteProcess(
        cmd=['ros2', 'run', 'control_1', 'topic_health_monitor_node', '--ros-args', '--params-file', robot_controllers],
        output='screen'
    )

    kasytwin_pose_bridge_spawner = Node(
        package='control_1',
        executable='kasytwin_pose_bridge.py',
        name='kasytwin_pose_bridge',
        output='screen',    
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )
    

    # Bridge between Gazebo and ROS2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/karo/pid/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/dev/ttyUSB0/differential_controller/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen',
        remappings=[
            ('/odometry', '/odom'),
        ]
    )





    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        node_robot_state_publisher,
        node_world_to_odom,
        odom_to_tf_node,
        gz_spawn_entity,
        pid_controller_spawner,
        reference_manager_spawner,
        motor_command_spawner,
        imu_quat_to_euler_spawner,
        laserscan_to_pointcloud_spawner,
        pca_node_spawner,
        health_monitor_process, 
        kasytwin_pose_bridge_spawner,
        gz_spawn_tube, 
        bridge,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])