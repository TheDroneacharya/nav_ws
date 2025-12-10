from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    offboard_commander_node = Node(
        package="warehouse_drone_commander",
        executable="offboard_commander_node",
        name="offboard_commander_node",
        output="screen",
        emulate_tty=True,
        parameters=[PathJoinSubstitution([FindPackageShare('warehouse_drone_commander'), 'config', 'offboard_commander_params.yaml'])],
        # arguments=["--ros-args", "--log-level", "DEBUG"],
    )
    setpoint_publisher_node = Node(
        package="warehouse_drone_commander",
        executable="setpoint_publisher_node",
        name="setpoint_publisher_node",
        output="screen",
        emulate_tty=True,
    )
    go_to_pose_action_server_node = Node(
        package="warehouse_drone_commander",
        executable="go_to_pose_action_server_node",
        name="go_to_pose_action_server_node",
        output="screen",
        emulate_tty=True,
        parameters=[PathJoinSubstitution([FindPackageShare('warehouse_drone_commander'), 'config', 'offboard_commander_params.yaml'])],
    )

    return LaunchDescription([
        offboard_commander_node,
        setpoint_publisher_node,
        go_to_pose_action_server_node,
    ])
