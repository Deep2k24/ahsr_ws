import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directories
    bringup_dir = get_package_share_directory('shopmate_bot_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Define the exact paths to your new files
    map_file = os.path.join(bringup_dir, 'maps', 'my_perfect_map.yaml')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        # 1. Start your hardware (Lidar, Odometry, URDF)
        # Make sure this launch file DOES NOT include slam_toolbox!
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'real_robot.launch.py'))
        ),

        # 2. Start the Nav2 Autonomous Brain
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'False',
                'autostart': 'True'
            }.items()
        ),

        # 3. Start RViz2 with the special Nav2 view
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])