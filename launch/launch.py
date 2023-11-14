from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beginner_tutorials',
            namespace='talker',
            executable='talker',
            name='flat_earther'
        ),
        Node(
            package='beginner_tutorials',
            namespace='listener',
            executable='listener',
            name='normal_person'
        )
    ])
