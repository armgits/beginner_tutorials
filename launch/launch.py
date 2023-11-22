from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


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

    record_bag_value = LaunchConfiguration('record_bag')
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='False',
        description="Record messages on all topics to a rosbag?"
    )

    first_astronaut_node = Node(
        package='beginner_tutorials',
        executable='first_astronaut',
        name='first_astronaut',
        parameters=[
            {"realization": realization_value}
        ]
    )

    second_astronaut_node = Node(
        package='beginner_tutorials',
        executable='second_astronaut',
        name='second_astronaut',
        parameters=[
            {"dramatic_end": dramatic_end_value}
        ]
    )

    record_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', 'output_bag'],
        output='screen',
        condition=IfCondition(PythonExpression([record_bag_value]))
    )
    
    return LaunchDescription([
        realization_arg,
        dramatic_end_arg,
        record_bag_arg,
        first_astronaut_node,
        second_astronaut_node,
        record_bag_process
    ])
