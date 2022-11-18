from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_sub',
            executable='publisher',
            parameters=[
                {'publish_message': 'New day'}
            ]
        ),
        Node(
            package='pub_sub',
            executable='subscriber'
        )
    ])