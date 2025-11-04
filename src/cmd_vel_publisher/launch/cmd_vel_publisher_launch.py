from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():    
    return LaunchDescription([
        DeclareLaunchArgument(name='cmd_vel_in', default_value='cmd_vel', description='Input cmd_vel topic (FLU frame)'),

        Node(
            package="cmd_vel_publisher",
            executable="cmd_vel_publisher",
            name="cmd_vel_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"cmd_vel_in": LaunchConfiguration('cmd_vel_in')},
            ]
        )
    ])
