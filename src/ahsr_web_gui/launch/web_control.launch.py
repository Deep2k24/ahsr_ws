from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Vosk model directory; empty uses ~/vosk-model-small-en-us-0.15',
    )

    return LaunchDescription([
        model_path_arg,
        
        # 1. Start the Voice Node
        Node(
            package='ahsr_web_gui',
            executable='techno_voice_node',
            name='techno_voice_node',
            output='screen',
            parameters=[{
                'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str),
                'tts_speed': 190,
            }],
        ),

        # 2. Start the Web GUI Node
        Node(
            package='ahsr_web_gui',
            executable='web_gui_node',
            name='web_gui_node',
            output='screen',
        )
    ])
