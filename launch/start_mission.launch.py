from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_explorer',
            # namespace='turtlesim1',
            executable='explorer',
            # name='sim'
        ),
        Node(
            package='custom_explorer',
            # namespace='turtlesim2',
            executable='missionControl',
            # name='sim'
        ),
    ])