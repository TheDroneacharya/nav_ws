#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.0',
        description='Target distance from wall/rack in meters'
    )
    
    use_depth_image_arg = DeclareLaunchArgument(
        'use_depth_image',
        default_value='true',
        description='Use depth image (true) or Range message (false)'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth_camera/depth',
        description='Depth image topic name'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wall_distance_controller'),
            'config',
            'wall_distance_params.yaml'
        ]),
        description='Path to the configuration YAML file'
    )

    # Wall Distance Controller Node
    wall_distance_controller_node = Node(
        package='wall_distance_controller',
        executable='wall_distance_controller_node',
        name='wall_distance_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'target_distance': LaunchConfiguration('target_distance'),
                'use_depth_image': LaunchConfiguration('use_depth_image'),
                'depth_image_topic': LaunchConfiguration('depth_topic'),
            }
        ],
        remappings=[
            # Add remappings  if needed
            # ('wall_distance/offset', '/drone/wall_offset'),
        ]
    )

    return LaunchDescription([
        target_distance_arg,
        use_depth_image_arg,
        depth_topic_arg,
        config_file_arg,
        wall_distance_controller_node,
    ])
