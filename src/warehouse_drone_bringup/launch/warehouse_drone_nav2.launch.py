from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	pkg_share = FindPackageShare(package='warehouse_drone_bringup').find('warehouse_drone_bringup')

	warehouse_drone_description = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('warehouse_drone_description'), 'launch', 'display.launch.py']),
		launch_arguments={
			'gui': 'False',
		}.items()
	)
	mavros = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'node.launch']),
		launch_arguments={
            'pluginlists_yaml': '$(pkg_share)/config/px4_pluginlists.yaml',
			'config_yaml': '$(pkg_share)/config/px4_config.yaml',
			'fcu_url': '/dev/ttyACM0:57600',
			'gcs_url': '',
			'tgt_system': '1',
			'tgt_component': '1',
			'log_output': 'screen',
			'fcu_protocol': 'v2.0',
			'respawn_mavros': 'false',
			'namespace': 'mavros',
        }.items(),
	)
	inspection_demo_launch = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('nav2_simple_commander'), 'inspection_demo_launch.py']),
		launch_arguments={
            'use_rviz': 'False',
            'use_slam': 'False',
        }.items(),
	)
	robot_pose_publisher = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('robot_pose_publisher_ros2'), 'launch', 'robot_pose_publisher_launch.py'])
	)
	relay_node = Node(
		package='topic_tools',
		executable='relay',
		parameters=[{
			'input_topic': '/robot_pose',
			'output_topic': '/mavros/vision_pose/pose'
		}]
	)
	return LaunchDescription([
		# warehouse_drone_description,
		# mavros,
		inspection_demo_launch,
		# robot_pose_publisher,
		# relay_node,
	])
