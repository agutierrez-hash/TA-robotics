from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collaborative_gen2',
            executable='server_node',
            output='screen',
            # Ya no necesitamos cargar nada extra
        )
    ])