from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # For changing ROS parameters
    topic_name_arg = DeclareLaunchArgument('topic_name', default_value = TextSubstitution(text="Messages"))
    publish_message_arg = DeclareLaunchArgument('publish_message', default_value = TextSubstitution(text="Terps Strong"))
    publish_interval_arg = DeclareLaunchArgument('publish_interval', default_value = TextSubstitution(text="1000"))

    return LaunchDescription([
        topic_name_arg,
        publish_message_arg,
        publish_interval_arg,
        Node(
            package='pub_sub',
            executable='publisher',
            parameters=[
                {"topic_name" : LaunchConfiguration('topic_name')},
                {"publish_message" : LaunchConfiguration('publish_message')},
                {"publish_interval" : LaunchConfiguration('publish_interval')}
            ]
        ),
        Node(
            package='pub_sub',
            executable='subscriber'
        )
    ])