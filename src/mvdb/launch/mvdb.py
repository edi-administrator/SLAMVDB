from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mvdb_tracker',
            namespace='mvdb_tracker',
            executable='tracker'
        ),
        Node(
            package='mvdb_mapper',
            namespace='mvdb_mapper',
            executable='mapper'
        ),
        Node(
            package='mvdb',
            executable='main'
        )
    ])