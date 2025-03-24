from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mvdb_tracker',
            namespace='mvdb_tracker',
            executable='tracker'
        ),
    ])