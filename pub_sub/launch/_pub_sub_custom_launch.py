from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # For changing ROS parameters
    topic_name_arg = DeclareLaunchArgument('topic_name', default_value = TextSubstitution(text="Messages"), description = "The name of the topic that publisher launches to.")

    publish_message_arg = DeclareLaunchArgument('publish_message', default_value = TextSubstitution(text="Terps Strong"), description = "The message that will be published in the topic.")

    publish_interval_arg = DeclareLaunchArgument('publish_interval', default_value = TextSubstitution(text="1000"), description = "Choose the publish interval for the publisher (in milli seconds).")

    ros_bag_choice = DeclareLaunchArgument('record_bag', default_value = TextSubstitution(text = "False"), choices = ['True', 'False'], description = "The argument that enables ros bag recording.")

    publisher_node = Node(
            package='pub_sub',
            executable='publisher',
            parameters=[
                {"topic_name" : LaunchConfiguration('topic_name')},
                {"publish_message" : LaunchConfiguration('publish_message')},
                {"publish_interval" : LaunchConfiguration('publish_interval')}
            ]
        )

    subscriber_node = Node(
            package = 'pub_sub',
            executable = 'subscriber',
            parameters = [
                {"topic_name" : LaunchConfiguration('topic_name')}
            ]
        ) 
    
    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
            cmd=['ros2', 'bag', 'record', '-a', '-d', '15'],
        shell=True
    )

    return LaunchDescription([
        topic_name_arg,
        publish_message_arg,
        publish_interval_arg,
        ros_bag_choice,
        publisher_node,
        subscriber_node,
        recorder
    ])