import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'shopmate_bot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. World File Path
    world_path = os.path.join(pkg_share, 'worlds', 'hospital.sdf')

    # 2. Xacro File Path
    xacro_file = os.path.join(pkg_share, 'urdf', 'shopmate_bot.xacro')
    robot_desc = os.popen(f'xacro {xacro_file}').read()

    # 3. Environment Variable (UPDATED FOR JAZZY/HARMONIC)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + pkg_share]
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, 
                    {'robot_description': robot_desc}],
    )

    # 5. Launch Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v4 {world_path}'}.items(),
    )

    # 6. Spawn the Robot
    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'shopmate_bot',
                   '-allow_renaming', 'true',
                   '-x', '-6.9030',
                   '-y', '-3.7388',
                   '-z', '0.02', 
                   '-Y', '0.0']
    )

    # 7. Bridge ROS <-> Gazebo Harmonic (UPDATED FOR JAZZY/HARMONIC)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        create_entity,
        bridge,
    ])