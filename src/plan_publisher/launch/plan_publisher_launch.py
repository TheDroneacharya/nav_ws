from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='plan_in', default_value='/plan', description='Input plan topic from planner'),
        DeclareLaunchArgument(name='plan_out', default_value='/plan_viz', description='Output (fixed) plan topic'),

        Node(
            package='plan_publisher',
            executable='plan_publisher',
            name='plan_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'plan_in': LaunchConfiguration('plan_in')},
                {'plan_out': LaunchConfiguration('plan_out')},
            ]
        )
    ])
