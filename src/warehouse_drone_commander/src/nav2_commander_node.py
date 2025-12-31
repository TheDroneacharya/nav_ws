#!/usr/bin/env python3
"""
Offboard Commander Node - Python Implementation
Converts functionality from offboard_commander_node.cpp to Python using rclpy
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from threading import Lock
import math
import time
from typing import Optional

# ROS 2 message imports
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import RCIn, State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool

# Custom action imports
from warehouse_drone_interfaces.action import GoToPose


class OffboardCommander(Node):
    """
    Offboard Commander Node for warehouse drone autonomous flight control.
    Manages arming, offboard mode, takeoff, and waypoint following.
    """

    def __init__(self, options=None):
        super().__init__("offboard_commander", options=options)
        
        self.get_logger().info("Offboard commander node constructed...")

        # ===================== Parameter Declarations =====================
        self.declare_parameter("setpoint_frame_id", "base_link")
        self.declare_parameter("takeoff_height", 1.0)
        self.declare_parameter("max_velocity_x", 1.0)
        self.declare_parameter("max_velocity_y", 1.0)
        self.declare_parameter("max_velocity_z", 1.0)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("go_to_pose_timeout", 60.0)

        self.declare_parameter("rc_arming_channel_index", 6)
        self.declare_parameter("rc_offboard_channel_index", 11)
        self.declare_parameter("rc_commandeered_channel_index", 12)
        self.declare_parameter("rc_total_channels", 18)
        self.declare_parameter("rc_arm_channel_threshold", 1300)
        self.declare_parameter("rc_offboard_channel_threshold", 1500)
        self.declare_parameter("rc_commandeered_channel_threshold", 1500)

        # Retrieve parameters
        self.setpoint_frame_id = self.get_parameter("setpoint_frame_id").value
        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.max_velocity_x = self.get_parameter("max_velocity_x").value
        self.max_velocity_y = self.get_parameter("max_velocity_y").value
        self.max_velocity_z = self.get_parameter("max_velocity_z").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.go_to_pose_timeout = self.get_parameter("go_to_pose_timeout").value

        self.arming_channel_index = self.get_parameter("rc_arming_channel_index").value - 1
        self.offboard_channel_index = self.get_parameter("rc_offboard_channel_index").value - 1
        self.commandeered_channel_index = self.get_parameter("rc_commandeered_channel_index").value - 1
        self.total_channels = self.get_parameter("rc_total_channels").value
        self.arm_channel_threshold = self.get_parameter("rc_arm_channel_threshold").value
        self.offboard_channel_threshold = self.get_parameter("rc_offboard_channel_threshold").value
        self.commandeered_channel_threshold = self.get_parameter("rc_commandeered_channel_threshold").value

        # ===================== Callback Groups =====================
        self.commander_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_callback_group = MutuallyExclusiveCallbackGroup()

        # ===================== Subscriptions and Publishers =====================
        sensor_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.rc_sub = self.create_subscription(
            RCIn,
            "/mavros/rc/in",
            self.rc_in_callback,
            qos_profile=sensor_qos
        )

        self.state_sub = self.create_subscription(
            State,
            "mavros/state",
            self.state_callback,
            qos_profile=sensor_qos
        )

        self.local_position_sub = self.create_subscription(
            PoseStamped,
            "mavros/local_position/pose",
            self.local_position_callback,
            qos_profile=sensor_qos
        )

        self.waypoint_target_sub = self.create_subscription(
            PoseStamped,
            "offboard_commander/input/waypoint_target",
            self.waypoint_target_callback,
            qos_profile=sensor_qos
        )

        self.waypoint_path_sub = self.create_subscription(
            Path,
            "waypoint_generator/path",
            self.waypoint_path_callback,
            qos_profile=sensor_qos
        )

        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            "offboard_commander/setpoint_raw/local",
            qos_profile=sensor_qos
        )

        self.arming_client = self.create_client(
            CommandBool,
            "mavros/cmd/arming"
        )

        self.set_mode_client = self.create_client(
            SetMode,
            "mavros/set_mode"
        )

        # ===================== Timers =====================
        self.loop_timer = self.create_timer(
            0.5,  # 500ms = 2 Hz
            self.loop_timer_callback_v2,
            callback_group=self.commander_callback_group
        )

        # ===================== Action Client =====================
        self.go_to_pose_action_client = ActionClient(
            self,
            GoToPose,
            "go_to_pose",
            callback_group=self.action_callback_group
        )

        # ===================== State Variables =====================
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_path = Path()
        self.setpoint_to_send = PositionTarget()
        self.current_goal_handle = None

        # ===================== Mutexes for Thread Safety =====================
        self.current_state_lock = Lock()
        self.current_pose_lock = Lock()
        self.target_pose_lock = Lock()
        self.target_path_lock = Lock()
        self.setpoint_to_send_lock = Lock()
        self.current_goal_handle_lock = Lock()

        # ===================== Flags (Atomic-like with Lock protection) =====================
        self.state_received = False
        self.pose_received = False
        self.waypoint_target_received = False
        self.waypoint_updated = False
        self.waypoint_path_received = False
        self.mission_received = False

        self.rc_connected = False
        self.rc_armed = False
        self.rc_offboard = False
        self.rc_commandeered = False

        self.to_set_offboard = False
        self.to_arm_vehicle = False
        self.to_takeoff = False

        self.startup_sequence_completed = False

        self.offboard_mode_set = False
        self.vehicle_armed = False
        self.in_flight = False

        self.holding_position = False
        self.in_mission = False

        # Lock for protecting flags (shared between callbacks and timer)
        self.flags_lock = Lock()

        # ===================== Logging Throttle Tracking =====================
        self.last_log_times = {}

    # ===================== Callback Methods =====================

    def state_callback(self, msg: State) -> None:
        """Handle MAVROS state updates."""
        with self.current_state_lock:
            self.current_state = msg

        with self.flags_lock:
            if not self.state_received:
                self.get_logger().info("First MAVROS state received.")
                self.state_received = True

    def rc_in_callback(self, msg: RCIn) -> None:
        """Handle RC input messages."""
        # Throttle logging to 5 seconds
        self._log_throttle(
            "RC Channels received: No. of channels: %d" % len(msg.channels),
            throttle_ms=5000,
            level="info"
        )

        if len(msg.channels) == self.total_channels:
            # RC previously disconnected, log connection
            with self.flags_lock:
                was_disconnected = not self.rc_connected
                self.rc_connected = True

            if was_disconnected:
                self.get_logger().info("RC connected.")

            # Extract channel values
            arm_channel_value = msg.channels[self.arming_channel_index]
            offboard_channel_value = msg.channels[self.offboard_channel_index]
            commandeered_channel_value = msg.channels[self.commandeered_channel_index]

            with self.flags_lock:
                # Check arming switch position
                if arm_channel_value > self.arm_channel_threshold:
                    if not self.rc_armed:
                        self.get_logger().info("Arming switch engaged.")
                        self.rc_armed = True
                else:
                    if self.rc_armed:
                        self.get_logger().info("Arming switch disengaged.")
                        self.rc_armed = False

                # Check offboard switch position
                if offboard_channel_value > self.offboard_channel_threshold:
                    if not self.rc_offboard:
                        self.get_logger().info("Offboard switch engaged.")
                        self.rc_offboard = True
                else:
                    if self.rc_offboard:
                        self.get_logger().info("Offboard switch disengaged.")
                        self.rc_offboard = False

                # Check commandeered switch position
                if commandeered_channel_value > self.commandeered_channel_threshold:
                    if not self.rc_commandeered:
                        self.get_logger().info("Commandeered switch engaged.")
                        self.rc_commandeered = True
                else:
                    if self.rc_commandeered:
                        self.get_logger().info("Commandeered switch disengaged.")
                        self.rc_commandeered = False

        else:
            # RC channel count mismatch
            with self.flags_lock:
                was_connected = self.rc_connected
                self.rc_connected = False

            if was_connected:
                self.get_logger().info("RC disconnected.")

            self.get_logger().error(
                "RC ERROR: Unexpected no. of channels! Expected: %d, Received: %d. Check param file!" 
                % (self.total_channels, len(msg.channels))
            )

    def local_position_callback(self, msg: PoseStamped) -> None:
        """Handle local position updates."""
        with self.current_pose_lock:
            self.current_pose = msg

        with self.flags_lock:
            if not self.pose_received:
                self.get_logger().info("First local pose received.")
                self.pose_received = True

    def waypoint_target_callback(self, msg: PoseStamped) -> None:
        """Handle waypoint target updates."""
        with self.flags_lock:
            if not self.mission_received:
                self.mission_received = True

                # Check if first waypoint target
                if not self.waypoint_target_received:
                    self.get_logger().info(
                        "First waypoint target received: (%.2f, %.2f, %.2f), yaw: %.2f rad"
                        % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                           self.quaternion_to_yaw(msg.pose.orientation))
                    )
                    with self.target_pose_lock:
                        self.target_pose = msg
                    self.waypoint_target_received = True
                else:
                    self.get_logger().info("Subsequent waypoint target received.")
            else:
                # In waypoint target mode or waypoint path mode
                if self.waypoint_target_received:
                    self.get_logger().info(
                        "Waypoint target updated: (%.2f, %.2f, %.2f), yaw: %.2f rad"
                        % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                           self.quaternion_to_yaw(msg.pose.orientation))
                    )
                    with self.target_pose_lock:
                        self.target_pose = msg
                    self.waypoint_updated = True
                else:
                    self.get_logger().info("Waypoint path mode - waypoint target ignored.")

    def waypoint_path_callback(self, msg: Path) -> None:
        """Handle waypoint path updates."""
        if msg.poses:
            with self.target_path_lock:
                self.target_path = msg
            self.get_logger().info("Received waypoint path with %d poses." % len(msg.poses))
        else:
            self.get_logger().error("Received empty waypoint path.")

    def setmode_response_callback(self, future) -> None:
        """Handle SetMode service response."""
        try:
            response = future.result()
            if response.mode_sent:
                with self.flags_lock:
                    self.offboard_mode_set = True
                self.get_logger().info("Offboard request successful.")
            else:
                self.get_logger().error("Offboard mode set request failed.")
        except Exception as e:
            self.get_logger().error("SetMode service call failed: %s" % str(e))

    def arming_response_callback(self, future) -> None:
        """Handle arming service response."""
        try:
            response = future.result()
            if response.success:
                with self.flags_lock:
                    self.vehicle_armed = True
                self.get_logger().info("Vehicle arm request successful. Result: %d" % response.success)
            else:
                self.get_logger().error("Vehicle arm request failed.")
        except Exception as e:
            self.get_logger().error("Arm service call failed: %s" % str(e))

    def go_to_pose_goal_response_callback(self, future) -> None:
        """Handle GoToPose goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("GoToPose goal was rejected by server.")
            with self.flags_lock:
                self.in_mission = False
                if self.to_takeoff and not self.in_flight:
                    self.to_takeoff = False
            return

        with self.current_goal_handle_lock:
            self.current_goal_handle = goal_handle

        with self.flags_lock:
            self.in_flight = True
            if not self.to_takeoff:
                self.in_mission = True

        self.get_logger().info("GoToPose goal accepted by server, waiting for result...")

    def go_to_pose_feedback_callback(self, feedback_msg) -> None:
        """Handle GoToPose feedback."""
        feedback = feedback_msg.feedback

        with self.current_pose_lock:
            current_pose_copy = self.current_pose

        self.get_logger().info(
            "GoToPose Feedback: Current Position: (%.2f, %.2f, %.2f), yaw: %.2f rad."
            % (current_pose_copy.pose.position.x, current_pose_copy.pose.position.y,
               current_pose_copy.pose.position.z,
               self.quaternion_to_yaw(current_pose_copy.pose.orientation))
        )
        self.get_logger().info(
            "GoToPose Feedback: Goal Position: (%.2f, %.2f, %.2f), yaw: %.2f rad."
            % (feedback.goal_pose.pose.position.x, feedback.goal_pose.pose.position.y,
               feedback.goal_pose.pose.position.z,
               self.quaternion_to_yaw(feedback.goal_pose.pose.orientation))
        )
        self.get_logger().info(
            "GoToPose Feedback: Distance remaining: %.2f m, Elapsed time: %.2f s"
            % (feedback.distance_remaining, feedback.elapsed_time)
        )

    def go_to_pose_result_callback(self, future) -> None:
        """Handle GoToPose result."""
        result = future.result()
        status = future.status

        if status == 4:  # SUCCEEDED
            with self.flags_lock:
                if self.to_takeoff:
                    self.get_logger().info(
                        "TAKEOFF SUCCESS: GoToPose takeoff action succeeded. "
                        "Now committing to hold position..."
                    )
                    self.hold_position()
                    self.to_takeoff = False
                    self.get_logger().info("TAKEOFF HOLD POSITION COMPLETE: hold_position returned after successful takeoff.")

            self.get_logger().info("GoToPose action succeeded.")
            self.get_logger().info("GoToPose Result: Success: %s" % result.success)

        elif status == 5:  # ABORTED
            self.get_logger().error("GoToPose action was aborted.")

        elif status == 6:  # CANCELED
            self.get_logger().warn("GoToPose action was canceled.")

        else:
            self.get_logger().error("Unknown result code for GoToPose action: %d" % status)

    def loop_timer_callback_v2(self) -> None:
        """Main control loop callback."""
        self.get_logger().info("Executing secondary loop timer callback.")

        # Check & interpret RC status
        if not self.check_rc_status():
            self._log_throttle(
                "RC not connected, skipping loop iteration...",
                throttle_ms=2000,
                level="warn"
            )
            return

        # Check, run, & wait for startup
        if not self.check_startup_status():
            self.get_logger().debug("loop_timer_callback_v2(): Startup sequence running...")
            return

        # Main control logic after startup sequence is completed
        self.get_logger().info("Startup sequence completed, entering main control loop.")

        with self.flags_lock:
            if self.mission_received:
                # If in mission, handle waypoint following
                self.get_logger().info("In mission, handling waypoint following...")

                if self.waypoint_target_received:
                    if self.waypoint_updated:
                        self.get_logger().info("Waypoint target updated, sending new goal...")
                        with self.current_goal_handle_lock:
                            goal_handle = self.current_goal_handle
                        self.update_waypoint(goal_handle)
                    else:
                        self.get_logger().info("Waypoint target not updated, continuing...")
                else:
                    self.get_logger().info("Waypoint path mode - following path...")
                    self.waypoint_handler()
            else:
                # If not in mission, hold position
                self.get_logger().info("Not in mission, holding position...")
                self.hold_position()

    # ===================== Core Functions =====================

    def startup_sequence_init(self) -> None:
        """Initialize startup sequence."""
        self.get_logger().info("Initiating startup sequence...")

        with self.current_state_lock:
            current_state_copy = self.current_state

        with self.flags_lock:
            if (self.state_received and current_state_copy.connected and 
                self.rc_connected and self.pose_received and not self.startup_sequence_completed):

                self.get_logger().info(
                    "MAVROS state received, RC connected, and local pose received. "
                    "Proceeding with startup sequence..."
                )

                # Publish setpoint - use current pose
                with self.current_pose_lock:
                    current_pose_copy = self.current_pose

                self.update_setpoint(current_pose_copy)
                self.publish_setpoint()

                # Attempt to set offboard mode
                if self.set_drone_ready_to_arm():
                    # Arm vehicle
                    if self.arm():
                        # Takeoff
                        if self.takeoff():
                            self.get_logger().info("Startup sequence completed.")
                            self.startup_sequence_completed = True
                        else:
                            self.get_logger().error("Takeoff failed during startup.")
                    else:
                        self.get_logger().error("Arming failed during startup.")
                else:
                    self.get_logger().error("Setting offboard mode failed during startup.")

            else:
                missing = []
                if not self.state_received:
                    missing.append("MAVROS state")
                if not current_state_copy.connected:
                    missing.append("MAVROS connection")
                if not self.rc_connected:
                    missing.append("RC connection")
                if not self.pose_received:
                    missing.append("local position")
                if self.startup_sequence_completed:
                    missing.append("startup already completed")

                self.get_logger().debug(
                    "Waiting for: %s" % ", ".join(missing)
                )

    def hold_position(self) -> None:
        """Hold the current position by setting setpoint to current position."""
        with self.flags_lock:
            if not self.pose_received:
                self.get_logger().error("Cannot hold position - pose not received yet.")
                return

        with self.current_pose_lock:
            current_pose_copy = self.current_pose

        self.update_setpoint(current_pose_copy)
        self.publish_setpoint()

        with self.flags_lock:
            self.holding_position = True

        self.get_logger().info("Holding position at (%.2f, %.2f, %.2f)" 
                              % (current_pose_copy.pose.position.x,
                                 current_pose_copy.pose.position.y,
                                 current_pose_copy.pose.position.z))

    def waypoint_handler(self) -> None:
        """Handle waypoint following logic."""
        with self.target_path_lock:
            path_copy = self.target_path

        if not path_copy.poses:
            self.get_logger().warn("No waypoints in path.")
            return

        for pose_stamped in path_copy.poses:
            self.get_logger().info(
                "Following waypoint: (%.2f, %.2f, %.2f)"
                % (pose_stamped.pose.position.x, pose_stamped.pose.position.y,
                   pose_stamped.pose.position.z)
            )
            self.go_to_pose_send_goal(pose_stamped)

    def update_waypoint(self, goal_handle) -> None:
        """Update waypoint target and cancel previous goal."""
        self.get_logger().info("Updating waypoint target, cancelling previous GoToPose goal...")

        cancel_success = self.go_to_pose_cancel_goal(goal_handle)
        if cancel_success:
            self.get_logger().info("GoToPose goal cancellation successful.")
        else:
            self.get_logger().error("GoToPose goal cancellation failed.")

        # Send new goal
        with self.target_pose_lock:
            target_pose_copy = self.target_pose

        self.go_to_pose_send_goal(target_pose_copy)
        with self.flags_lock:
            self.waypoint_updated = False

    # ===================== Utility Functions =====================

    def check_startup_status(self) -> bool:
        """Check if startup sequence is completed."""
        with self.flags_lock:
            if not self.startup_sequence_completed:
                # Run startup sequence
                self.startup_sequence_init()
                return False
            else:
                return True

    def check_rc_status(self) -> bool:
        """Check if RC is connected and interpret RC inputs."""
        with self.flags_lock:
            if not self.rc_connected:
                return False

            # Set control flags based on RC switches
            if self.rc_armed:
                self.to_arm_vehicle = True
            else:
                self.to_arm_vehicle = False

            if self.rc_offboard:
                self.to_set_offboard = True
            else:
                self.to_set_offboard = False

            if self.rc_commandeered:
                if not self.to_takeoff:
                    self.to_takeoff = True
            else:
                # Don't unset to_takeoff to prevent re-triggering
                pass

            return True

    def check_drone_vitals_status(self) -> bool:
        """Check various drone vitals."""
        with self.flags_lock:
            if self.state_received:
                with self.current_state_lock:
                    current_state_copy = self.current_state

                if not current_state_copy.connected:
                    self.get_logger().warn("MAVROS not connected to drone.")
                    return False

                if not current_state_copy.armed and self.to_arm_vehicle:
                    self.get_logger().info("Drone not armed but arm requested.")

                return True
            else:
                self.get_logger().error("MAVROS state not received.")
                return False

    def set_drone_ready_to_arm(self) -> bool:
        """Set drone to offboard mode."""
        with self.flags_lock:
            if self.to_set_offboard:
                if not self.offboard_mode_set:
                    self.get_logger().info("Setting offboard mode...")

                    request = SetMode.Request()
                    request.custom_mode = "OFFBOARD"

                    future = self.set_mode_client.call_async(request)
                    future.add_done_callback(self.setmode_response_callback)

                    # Wait for response with timeout
                    timeout_start = time.time()
                    while not self.offboard_mode_set and (time.time() - timeout_start) < 5.0:
                        time.sleep(0.1)

                    if self.offboard_mode_set:
                        self.get_logger().info("Offboard mode set successfully.")
                        return True
                    else:
                        self.get_logger().error("Failed to set offboard mode.")
                        return False
                else:
                    return True
            else:
                return False

    def arm(self) -> bool:
        """Arm the vehicle."""
        with self.flags_lock:
            if self.to_arm_vehicle:
                if not self.vehicle_armed:
                    self.get_logger().info("Arming vehicle...")

                    request = CommandBool.Request()
                    request.value = True

                    future = self.arming_client.call_async(request)
                    future.add_done_callback(self.arming_response_callback)

                    # Wait for response with timeout
                    timeout_start = time.time()
                    while not self.vehicle_armed and (time.time() - timeout_start) < 5.0:
                        time.sleep(0.1)

                    if self.vehicle_armed:
                        self.get_logger().info("Vehicle armed successfully.")
                        return True
                    else:
                        self.get_logger().error("Failed to arm vehicle.")
                        return False
                else:
                    return True
            else:
                return False

    def takeoff(self) -> bool:
        """Execute takeoff sequence."""
        with self.flags_lock:
            if self.to_takeoff:
                if not self.in_flight:
                    self.get_logger().info("Initiating takeoff sequence...")

                    with self.current_pose_lock:
                        current_pose_copy = self.current_pose

                    takeoff_pose = PoseStamped()
                    takeoff_pose.header = current_pose_copy.header
                    takeoff_pose.pose.position.x = current_pose_copy.pose.position.x
                    takeoff_pose.pose.position.y = current_pose_copy.pose.position.y
                    takeoff_pose.pose.position.z = current_pose_copy.pose.position.z + self.takeoff_height
                    takeoff_pose.pose.orientation = current_pose_copy.pose.orientation

                    self.go_to_pose_send_goal(takeoff_pose)

                    # Wait for in_flight flag with timeout
                    timeout_start = time.time()
                    while not self.in_flight and (time.time() - timeout_start) < self.go_to_pose_timeout:
                        time.sleep(0.1)

                    if self.in_flight:
                        self.get_logger().info("Takeoff initiated successfully.")
                        return True
                    else:
                        self.get_logger().error("Takeoff failed to initiate.")
                        return False
                else:
                    return True
            else:
                return False

    def update_setpoint(self, pose: Optional[PoseStamped] = None) -> None:
        """Update the setpoint message with desired values."""
        if pose is not None:
            with self.setpoint_to_send_lock:
                setpoint = self.populate_setpoint(pose)
                self.setpoint_to_send = setpoint
        else:
            self.get_logger().debug("update_setpoint called with None pose.")

    def publish_setpoint(self) -> None:
        """Publish the current setpoint."""
        with self.setpoint_to_send_lock:
            self.setpoint_to_send.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.setpoint_to_send)

    def populate_setpoint(self, pose: PoseStamped) -> PositionTarget:
        """Create a PositionTarget message from a PoseStamped."""
        setpoint_msg = PositionTarget()
        setpoint_msg.header.frame_id = self.setpoint_frame_id
        setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Type mask: use position, velocity, yaw, yaw_rate; ignore acceleration/force
        setpoint_msg.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ
        )
        
        setpoint_msg.position.x = pose.pose.position.x
        setpoint_msg.position.y = pose.pose.position.y
        setpoint_msg.position.z = pose.pose.position.z
        setpoint_msg.velocity.x = self.max_velocity_x
        setpoint_msg.velocity.y = self.max_velocity_y
        setpoint_msg.velocity.z = self.max_velocity_z
        setpoint_msg.acceleration_or_force.x = 0.0
        setpoint_msg.acceleration_or_force.y = 0.0
        setpoint_msg.acceleration_or_force.z = 0.0
        setpoint_msg.yaw = self.quaternion_to_yaw(pose.pose.orientation)
        setpoint_msg.yaw_rate = self.max_yaw_rate

        return setpoint_msg

    def go_to_pose_send_goal(self, target_pose: PoseStamped) -> None:
        """Send a goal to the GoToPose action server."""
        self.get_logger().debug("go_to_pose_send_goal(): Received request to send GoToPose goal...")

        # Wait for action server
        self.get_logger().debug("go_to_pose_send_goal(): Waiting for GoToPose server for up to 5s...")
        if not self.go_to_pose_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("GoToPose action server not available.")
            return

        goal_msg = GoToPose.Goal()
        goal_msg.goal_pose = target_pose
        goal_msg.max_velocity_x = self.max_velocity_x
        goal_msg.max_velocity_y = self.max_velocity_y
        goal_msg.max_velocity_z = self.max_velocity_z
        goal_msg.max_yaw_rate = self.max_yaw_rate

        self.get_logger().info(
            "Sending GoToPose goal to position: (%.2f, %.2f, %.2f), yaw: %.2f rad"
            % (target_pose.pose.position.x, target_pose.pose.position.y,
               target_pose.pose.position.z,
               self.quaternion_to_yaw(target_pose.pose.orientation))
        )

        future = self.go_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.go_to_pose_feedback_callback
        )
        future.add_done_callback(self.go_to_pose_goal_response_callback)

        # Add result callback
        def add_result_callback(future_inner):
            goal_handle = future_inner.result()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.go_to_pose_result_callback)

        future.add_done_callback(add_result_callback)

    def go_to_pose_cancel_goal(self, goal_handle) -> bool:
        """Cancel a GoToPose goal."""
        self.get_logger().debug("go_to_pose_cancel_goal(): Received request to cancel GoToPose goal...")

        if not goal_handle:
            self.get_logger().warn("go_to_pose_cancel_goal(): No goal handle provided.")
            return False

        self.get_logger().debug("go_to_pose_cancel_goal(): Attempting to cancel GoToPose goal...")

        try:
            cancel_future = goal_handle.cancel_goal_async()
            # Wait for cancellation response
            cancel_future.result()
            self.get_logger().info("GoToPose goal cancellation request sent.")
            return True
        except Exception as e:
            self.get_logger().error("Failed to cancel GoToPose goal: %s" % str(e))
            return False

    @staticmethod
    def quaternion_to_yaw(quaternion: Quaternion) -> float:
        """Extract yaw angle from quaternion (output in radians)."""
        # Yaw (z-axis rotation) from quaternion
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _log_throttle(self, message: str, throttle_ms: int = 1000, level: str = "info") -> None:
        """Throttle logging messages to avoid spam."""
        current_time = time.time() * 1000  # Convert to milliseconds

        if message not in self.last_log_times:
            self.last_log_times[message] = 0.0

        if (current_time - self.last_log_times[message]) >= throttle_ms:
            self.last_log_times[message] = current_time
            if level == "info":
                self.get_logger().info(message)
            elif level == "warn":
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)
            elif level == "debug":
                self.get_logger().debug(message)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    node_options = rclpy.node.NodeOptions()
    offboard_commander_node = OffboardCommander(node_options)

    executor = MultiThreadedExecutor()
    executor.add_node(offboard_commander_node)

    offboard_commander_node.get_logger().info("Starting Offboard Commander Node...")

    try:
        executor.spin()
    except KeyboardInterrupt:
        offboard_commander_node.get_logger().info("Shutting down Offboard Commander Node...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
