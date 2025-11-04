from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
	pkg_share = FindPackageShare(package='warehouse_drone_bringup').find('warehouse_drone_bringup')

	mavros = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'node.launch']),
		launch_arguments={
            'pluginlists_yaml': '$(pkg_share)/config/px4_pluginlists.yaml',
			'config_yaml': '$(pkg_share)/config/px4_config.yaml',
			'fcu_url': '/dev/ttyTHS1:57600',
			'gcs_url': '',
			'tgt_system': '1',
			'tgt_component': '1',
			'log_output': 'screen',
			'fcu_protocol': 'v2.0',
			'respawn_mavros': 'false',
			'namespace': 'mavros',
        }.items(),
	)
	nvblox = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('nvblox_examples_bringup'), 'launch', 'realsense_example.launch.py']),
		launch_arguments={
            'run_foxglove': 'True',
			'run_rviz': 'False',
        }.items(),
	)
	foxglove = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])
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
		# mavros,
		nvblox,
		foxglove,
		# robot_pose_publisher,
		# relay_node,
	])
