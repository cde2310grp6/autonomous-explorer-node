from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_explorer',
            # namespace='explorer',
            executable='explorer',
            # name='PLEASWORK1'
            output='screen', 
        ),
        Node(
            package='custom_explorer',
            # namespace='mission_execute',
            executable='mission_GOOOO',
            # name='PLEASWORK2'
            output='screen', 
        ),
    ])