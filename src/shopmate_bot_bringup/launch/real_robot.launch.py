import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package Directories
    pkg_shopmate_description = get_package_share_directory('shopmate_bot_description')
    pkg_rplidar = get_package_share_directory('rplidar_ros')

    # 1. Process the URDF file
    xacro_file = os.path.join(pkg_shopmate_description, 'urdf', 'shopmate_bot.xacro')
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # 2. Robot State Publisher (Loads the TF tree for the real robot)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}] 
    )

    # 3. Joint State Publisher (Fills in blank joint data so the robot model renders!)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # 4. RPLidar C1 Node
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_9e2413cd467fef1180a4221cedd322a4-if00-port0', 
            'frame_id': 'lidar_link_1'
        }.items()
    )

    # 5. Wheel Odometry Node (CRITICAL FOR NAV2)
    # This runs your custom C++ Arduino bridge
    odom_node = Node(
        package='wheel_odom_cpp', 
        executable='wheel_odom_node', 
        name='wheel_odom_node',
        output='screen'
    )

    # Note: RViz2 was removed from here because navigation.launch.py will handle it!

    return LaunchDescription([
        rsp_node,
        jsp_node,
        lidar_launch,
        odom_node
    ])