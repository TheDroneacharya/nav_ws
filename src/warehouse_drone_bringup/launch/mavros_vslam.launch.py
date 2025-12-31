from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import math

def generate_launch_description():
	pkg_share = FindPackageShare(package='warehouse_drone_bringup').find('warehouse_drone_bringup')
	
	# launch file arguments
	mavros = LaunchConfiguration('mavros')
	foxglove = LaunchConfiguration('foxglove')
	nav2 = LaunchConfiguration('nav2')
	cmd_vel_enu = LaunchConfiguration('cmd_vel_enu')
	# nvblox launch file arguments
	foxglove_nvblox = LaunchConfiguration('foxglove_nvblox')
	rviz_nvblox = LaunchConfiguration('rviz_nvblox')

	# params_file = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'params', 'navigation.yaml'])

	# param_substitutions = {
	# 	'default_nav_to_pose_bt_xml': PathJoinSubstitution([FindPackageShare('warehouse_drone_bringup'), 'behavior_trees', 'test_version_navigate_to_pose_default.xml']),
	# 	'default_nav_through_poses_bt_xml': PathJoinSubstitution([FindPackageShare('warehouse_drone_bringup'), 'behavior_trees', 'test_version_navigate_through_poses_default.xml']),
	# }

	# configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,
    #         param_rewrites=param_substitutions,
    #         convert_types=True),
    #     allow_substs=True)

	warehouse_drone_description = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('warehouse_drone_description'), 'launch', 'display.launch.py']),
		launch_arguments={
			'gui': 'False',
		}.items(),
	)
	foxglove = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml']),
		condition=IfCondition(PythonExpression([foxglove, ' and not ', foxglove_nvblox])),
	)
	nvblox = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('nvblox_examples_bringup'), 'launch', 'realsense_example.launch.py']),
		launch_arguments={
            'run_foxglove': LaunchConfiguration('foxglove_nvblox'),
			'run_rviz': LaunchConfiguration('rviz_nvblox'),
        }.items(),
	)
	map_static_transform_publisher = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_to_enu',
		output='screen',
        arguments=['0', '0', '0', '0', '0', PythonExpression(str(math.sin(-math.pi/4))), PythonExpression(str(math.cos(-math.pi/4))), 'map_vslam', 'map'],
	)
	odom_static_transform_publisher = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='odom_to_enu',
		output='screen',
        arguments=['0', '0', '0', '0', '0', PythonExpression(str(math.sin(-math.pi/4))), PythonExpression(str(math.cos(-math.pi/4))), 'odom_vslam', 'odom'],
	)
	mavros = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'node.launch']),
		launch_arguments={
            'pluginlists_yaml': PathJoinSubstitution([pkg_share, 'config', 'px4_pluginlists.yaml']),
			'config_yaml': PathJoinSubstitution([pkg_share, 'config', 'px4_config.yaml']),
			'fcu_url': '/dev/ttyTHS1:921600',
			'gcs_url': '',
			'tgt_system': '1',
			'tgt_component': '1',
			'log_output': 'screen',
			'fcu_protocol': 'v2.0',
			'respawn_mavros': 'false',
			'namespace': 'mavros',
        }.items(),
		condition=IfCondition(LaunchConfiguration('mavros')),
	)
	robot_pose_publisher = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('robot_pose_publisher_ros2'), 'launch', 'robot_pose_publisher_launch.py']),
	)
	pose_relay_node = Node(
		package='topic_tools',
		executable='relay',
		parameters=[{
			'input_topic': '/robot_pose',
			'output_topic': '/mavros/vision_pose/pose'
		}]
	)
	odom_relay_node = Node(
		package='topic_tools',
		executable='relay',
		parameters=[{
			'input_topic': '/visual_slam/tracking/odometry',
			'output_topic': '/odom'
		}]
	)
	cmd_vel_publisher = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('cmd_vel_publisher'), 'launch', 'cmd_vel_publisher_launch.py']),
		launch_arguments={
            'cmd_vel_in': 'cmd_vel',
        }.items(),
		condition=IfCondition(LaunchConfiguration('cmd_vel_enu')),
	)
	cmd_vel_enu_relay_node = Node(
		package='topic_tools',
		executable='relay',
		parameters=[{
			'input_topic': '/cmd_vel_enu',
			'output_topic': '/mavros/setpoint_velocity/cmd_vel_unstamped'
		}],
		condition=IfCondition(LaunchConfiguration('cmd_vel_enu'))
	)
	cmd_vel_relay_node = Node(
		package='topic_tools',
		executable='relay',
		parameters=[{
			'input_topic': '/cmd_vel',
			'output_topic': '/mavros/setpoint_velocity/cmd_vel_unstamped'
		}],
		condition=UnlessCondition(LaunchConfiguration('cmd_vel_enu'))
	)
	inspection_demo_launch = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('nav2_simple_commander'), 'inspection_demo_launch.py']),
		launch_arguments={
            'use_rviz': 'False',
            'use_slam': 'False',
			# 'params_file': PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'params', 'navigation.yaml']),
			'params_file': PathJoinSubstitution([FindPackageShare('warehouse_drone_bringup'), 'config', 'navigation.yaml']),
        }.items(),
		condition=IfCondition(LaunchConfiguration('nav2'))
	)
	plan_publisher = IncludeLaunchDescription(
		PathJoinSubstitution([FindPackageShare('plan_publisher'), 'launch', 'plan_publisher_launch.py']),
		condition=IfCondition(LaunchConfiguration('nav2'))
	)

	return LaunchDescription([
		DeclareLaunchArgument(name='foxglove', default_value='True', description='Flag to launch foxglove bridge node seperately'),
		DeclareLaunchArgument(name='foxglove_nvblox', default_value='False', description='Flag to launch foxglove bridge node in nvblox launch file'),
		DeclareLaunchArgument(name='rviz_nvblox', default_value='False', description='Flag to launch rviz node in nvblox launch file'),
		DeclareLaunchArgument(name='mavros', default_value='False', description='Flag to launch mavros launch file'),
		DeclareLaunchArgument(name='nav2', default_value='False', description='Flag to launch nav2 inspection demo launch file'),
		DeclareLaunchArgument(name='cmd_vel_enu', default_value='False', description='Source of cmd_vel commands. If true, cmd_vel_publisher node will be launched'),
		warehouse_drone_description,
		foxglove,
		nvblox,
		map_static_transform_publisher,
		odom_static_transform_publisher,
		mavros,
		robot_pose_publisher,
		pose_relay_node,
		odom_relay_node,
		cmd_vel_publisher,
		cmd_vel_enu_relay_node,
		cmd_vel_relay_node,
		inspection_demo_launch,
		plan_publisher,
	])
