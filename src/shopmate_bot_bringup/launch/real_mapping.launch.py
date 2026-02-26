import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    # Package Directories
    pkg_shopmate_description = get_package_share_directory('shopmate_bot_description')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # 1. Process the URDF file
    xacro_file = os.path.join(pkg_shopmate_description, 'urdf', 'shopmate_bot.xacro')
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # RViz Configuration Path
    rviz_config_path = os.path.join(pkg_shopmate_description, 'config', 'display.rviz')

    # 2. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}] 
    )

    # 3. Joint State Publisher (Keeps wheels attached in RViz)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # 4. Wheel Odometry Node 
    odom_node = Node(
        package='wheel_odom_cpp',
        executable='wheel_odom_node',
        output='screen'
    )

    # 5. RPLidar Node (Using permanent Lidar ID)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0', 
            'frame_id': 'lidar_link_1'
        }.items()
    )

    # 6. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'False'}.items()
    )

    # 7. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}] 
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        odom_node,
        lidar_launch,
        TimerAction(period=2.0, actions=[slam_launch]),
        TimerAction(period=3.0, actions=[rviz_node])
    ])