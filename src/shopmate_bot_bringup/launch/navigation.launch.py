import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- 1. Package Directories ---
    pkg_shopmate_bringup = get_package_share_directory('shopmate_bot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_shopmate_voice = get_package_share_directory('shopmate_bot_voice') # 🟢 Voice Package Added

    # --- 2. Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # --- 3. Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', # Set to false for the real robot
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_shopmate_bringup, 'maps', 'my_perfect_map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_shopmate_bringup, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # --- 4. Main Nav2 Bringup Launch ---
    # We disable extra nodes to prevent crashes and ensure a clean hospital navigation stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'true',
            'use_respawn': 'false',
            # Disable extra modules causing issues
            'use_docking': 'false',
            'use_collision_monitor': 'false',
            'use_velocity_smoother': 'false'
        }.items()
    )

    # Wrap the Nav2 launch in a TimerAction to give AMCL time to load the map-to-odom transform
    delayed_nav2_launch = TimerAction(
        period=6.0,
        actions=[nav2_launch]
    )

    # --- 5. 🤖 Techno AI Voice Assistant ---
    # Boots up the LLM and Vosk microphone listener
    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_shopmate_voice, 'techno_voice.launch.py')
        )
    )

    # --- 6. Return Launch Description ---
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        delayed_nav2_launch,
        voice_launch  # 🟢 Voice Node added here!
    ])