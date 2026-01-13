from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Serial Bridge (Talks to Arduino)
        Node(
            package='agribot_precision_sprayer',
            executable='bridge',
            name='serial_bridge',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud': 115200}
            ]
        ),
        
        # 2. Vision Detector
        Node(
            package='agribot_precision_sprayer',
            executable='detector',
            name='weed_detector',
            output='screen'
        ),
        
        # 3. Logic Manager
        Node(
            package='agribot_precision_sprayer',
            executable='manager',
            name='weed_manager',
            output='screen'
        )
    ])
