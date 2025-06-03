from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the rover_main node
        Node(
            package='rover_core',
            executable='rover_main',
            name='rover_main_node',
            output='screen'
        ),
        Node(
            package='rover_core',
            executable='drive_unit',
            name='drive_unit_node',
            output='screen'
        )
    ])

