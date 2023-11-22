from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    realization_value = LaunchConfiguration('realization')
    realization_arg = DeclareLaunchArgument(
        'realization',
        default_value="Wait, the earth's round?",
        description="What did the first astronaut realize?"
    )

    dramatic_end_value = LaunchConfiguration('dramatic_end')
    dramatic_end_arg = DeclareLaunchArgument(
        'dramatic_end',
        default_value='false',
        description="Should the second astronaut end up shooting the first \
                                               astronaut for a dramatic ending?"
    )

    first_astronaut_node = Node(
        package='beginner_tutorials',
        namespace='space',
        executable='first_astronaut',
        name='first_astronaut',
        parameters=[
            {"realization": realization_value}
        ]
    )

    second_astronaut_node = Node(
        package='beginner_tutorials',
        namespace='space',
        executable='second_astronaut',
        name='second_astronaut',
        parameters=[
            {"dramatic_end": dramatic_end_value}
        ]
    )

    return LaunchDescription([
        realization_arg,
        dramatic_end_arg,
        first_astronaut_node,
        second_astronaut_node
    ])
