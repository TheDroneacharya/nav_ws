#!/usr/bin/python3
"""
Demo Simulation Script for Waypoint System

This script simulates:
1. Battery draining (1% per second)
2. Random wall/depth distance (1.0m ± 0.25m)
3. Full waypoint mission execution
4. RTL trigger when battery is low
5. Battery charging during RTL
6. Mission resume after charging

Usage:
    ros2 run waypoint_generator demo_simulation.py

Make sure waypoint_server is running first:
    ros2 run waypoint_generator waypoint_server
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import BatteryState, Range
from geometry_msgs.msg import Vector3Stamped
from diagnostic_msgs.msg import DiagnosticStatus
from waypoint_interfaces.action import InspectRack, ReturnToLaunch
from waypoint_interfaces.msg import MissionState
from waypoint_interfaces.srv import ResumeMission
import random
import time
import threading


class DemoSimulation(Node):
    def __init__(self):
        super().__init__('demo_simulation')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # State
        self.battery_level = 100.0
        self.is_charging = False
        self.mission_running = False
        self.rtl_triggered = False
        self.mission_completed = False
        self.current_mission_state = MissionState.STATE_IDLE
        
        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.depth_pub = self.create_publisher(Range, '/depth_camera/range', 10)
        self.wall_offset_pub = self.create_publisher(Vector3Stamped, '/wall_distance/offset', 10)
        self.health_pub = self.create_publisher(DiagnosticStatus, '/system_health', 10)
        
        # Subscribers
        self.mission_state_sub = self.create_subscription(
            MissionState,
            '/mission_state',
            self.mission_state_callback,
            10
        )
        
        # Action clients
        self.inspect_client = ActionClient(
            self, InspectRack, 'inspect_rack',
            callback_group=self.callback_group
        )
        self.rtl_client = ActionClient(
            self, ReturnToLaunch, 'return_to_launch',
            callback_group=self.callback_group
        )
        
        # Service client
        self.resume_client = self.create_client(
            ResumeMission, 'resume_mission',
            callback_group=self.callback_group
        )
        
        # Timers - battery drains every 3.5 seconds to match waypoint timing
        self.battery_timer = self.create_timer(3.5, self.battery_update)
        self.sensor_timer = self.create_timer(0.1, self.sensor_update)
        self.health_timer = self.create_timer(1.0, self.health_update)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('       DRONE SIMULATION DEMO STARTED')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Battery Level: 100%')
        self.get_logger().info('System Status: Healthy')
        self.get_logger().info('')
        self.get_logger().info('Starting mission in 3 seconds...')
        
        # Start mission after delay
        self.startup_timer = self.create_timer(3.0, self.start_mission)
        
    def mission_state_callback(self, msg):
        """Track mission state from waypoint server"""
        self.current_mission_state = msg.state
        
    def battery_update(self):
        """Update battery level - drain or charge"""
        if self.is_charging:
            # Charging at 5% per second
            self.battery_level = min(100.0, self.battery_level + 5.0)
            if self.battery_level >= 80.0:
                self.get_logger().info(f'🔋 Battery charged to {self.battery_level:.0f}% - Ready to resume!')
                self.is_charging = False
                # Resume mission after charging
                self.resume_mission()
        else:
            # Drain at 1% per second when mission running
            if self.mission_running:
                self.battery_level = max(0.0, self.battery_level - 4.0)
                
            if self.battery_level <= 20.0 and not self.rtl_triggered and self.mission_running:
                self.get_logger().warning(f'⚠️ LOW BATTERY: {self.battery_level:.0f}% - RTL will trigger!')
        
        # Publish battery state
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = self.battery_level / 100.0  # Convert to 0.0-1.0 range
        msg.voltage = 11.1 + (self.battery_level / 100.0) * 1.5  # Simulate voltage
        msg.present = True
        self.battery_pub.publish(msg)
        
        # Log battery every 10%
        if int(self.battery_level) % 10 == 0 and self.mission_running:
            self.get_logger().info(f'🔋 Battery: {self.battery_level:.0f}%')
            
    def sensor_update(self):
        """Publish simulated sensor data"""
        # Simulate wall/depth distance with noise: 1.0m ± 0.25m
        distance = 1.0 + random.uniform(-0.25, 0.25)
        
        # Publish as Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'depth_camera_link'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.5
        range_msg.min_range = 0.1
        range_msg.max_range = 10.0
        range_msg.range = distance
        self.depth_pub.publish(range_msg)
        
        # Calculate wall offset correction (target is 1.0m, with P=0.5)
        error = distance - 1.0
        correction = -0.5 * error
        correction = max(-0.3, min(0.3, correction))  # Clamp to ±0.3m
        
        # Publish offset correction
        offset_msg = Vector3Stamped()
        offset_msg.header.stamp = self.get_clock().now().to_msg()
        offset_msg.header.frame_id = 'base_link'
        offset_msg.vector.x = 0.0
        offset_msg.vector.y = correction
        offset_msg.vector.z = 0.0
        self.wall_offset_pub.publish(offset_msg)
        
    def health_update(self):
        """Publish healthy system status"""
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.OK
        msg.name = 'System Health'
        msg.message = 'All systems operational'
        self.health_pub.publish(msg)
        
    def start_mission(self):
        """Start the inspection mission"""
        self.startup_timer.cancel()
        
        self.get_logger().info('')
        self.get_logger().info('🚀 STARTING INSPECTION MISSION')
        self.get_logger().info('   Rack ID: rack_1')
        self.get_logger().info('   Wall: 2.0m x 2.0m')
        self.get_logger().info('   Working Distance: 0.5m')
        self.get_logger().info('')
        
        # Wait for action server
        if not self.inspect_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('InspectRack action server not available!')
            return
            
        # Send goal
        goal = InspectRack.Goal()
        goal.rack_id = 'rack_1'
        goal.wall_width = 0.0  # Use config values
        goal.wall_height = 0.0
        goal.working_distance = 0.0
        
        self.mission_running = True
        self.rtl_triggered = False
        
        send_goal_future = self.inspect_client.send_goal_async(
            goal, 
            feedback_callback=self.inspect_feedback
        )
        send_goal_future.add_done_callback(self.inspect_goal_response)
        
    def inspect_goal_response(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
            
        self.get_logger().info('✅ Goal accepted - executing waypoints...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.inspect_result)
        
    def inspect_feedback(self, feedback_msg):
        """Handle mission feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'📍 Waypoint {fb.waypoint_index + 1}: '
            f'({fb.current_waypoint.x:.2f}, {fb.current_waypoint.y:.2f}, {fb.current_waypoint.z:.2f}) '
            f'- {fb.percentage_complete:.0f}%'
        )
        
    def inspect_result(self, future):
        """Handle mission result"""
        result = future.result().result
        self.mission_running = False
        
        if result.success:
            self.get_logger().info('')
            self.get_logger().info('=' * 60)
            self.get_logger().info('✅ MISSION COMPLETED SUCCESSFULLY!')
            self.get_logger().info(f'   Total Waypoints: {result.total_waypoints}')
            self.get_logger().info('=' * 60)
            self.mission_completed = True
        else:
            self.get_logger().warning(f'⚠️ Mission interrupted: {result.message}')
            if 'RTL' in result.message:
                self.rtl_triggered = True
                self.get_logger().info('')
                self.get_logger().info('🏠 EXECUTING RETURN TO LAUNCH...')
                self.execute_rtl()
                
    def execute_rtl(self):
        """Execute RTL action"""
        if not self.rtl_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('RTL action server not available!')
            return
            
        goal = ReturnToLaunch.Goal()
        goal.takeoff_position.x = 0.0
        goal.takeoff_position.y = 0.0
        goal.takeoff_position.z = 0.0
        goal.rtl_altitude = 2.0
        
        send_goal_future = self.rtl_client.send_goal_async(
            goal,
            feedback_callback=self.rtl_feedback
        )
        send_goal_future.add_done_callback(self.rtl_goal_response)
        
    def rtl_goal_response(self, future):
        """Handle RTL goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('RTL goal rejected!')
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rtl_result)
        
    def rtl_feedback(self, feedback_msg):
        """Handle RTL feedback"""
        fb = feedback_msg.feedback
        states = ['ASCENDING', 'FLYING_HOME', 'DESCENDING', 'LANDED']
        state_name = states[fb.rtl_state] if fb.rtl_state < len(states) else 'UNKNOWN'
        self.get_logger().info(f'🏠 RTL: {state_name} - Distance to home: {fb.distance_to_home:.1f}m')
        
    def rtl_result(self, future):
        """Handle RTL result"""
        result = future.result().result
        if result.success:
            self.get_logger().info('')
            self.get_logger().info('✅ RTL COMPLETED - Drone at home position')
            self.get_logger().info(f'   Final position: ({result.final_position.x:.1f}, {result.final_position.y:.1f}, {result.final_position.z:.1f})')
            self.get_logger().info('')
            self.get_logger().info('🔌 CHARGING BATTERY...')
            self.is_charging = True
        else:
            self.get_logger().error(f'RTL failed: {result.message}')
            
    def resume_mission(self):
        """Resume mission from last waypoint"""
        self.get_logger().info('')
        self.get_logger().info('🔄 RESUMING MISSION...')
        
        if not self.resume_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Resume service not available!')
            # Start fresh mission instead
            self.start_mission()
            return
            
        request = ResumeMission.Request()
        request.resume = True
        request.start_from_waypoint = 0  # Use last saved waypoint
        request.mission_id = ''
        
        future = self.resume_client.call_async(request)
        future.add_done_callback(self.resume_response)
        
    def resume_response(self, future):
        """Handle resume response"""
        result = future.result()
        if result.success:
            self.get_logger().info(f'✅ Resuming from waypoint {result.resumed_from_waypoint}')
            self.get_logger().info(f'   Remaining waypoints: {result.remaining_waypoints}')
            # Restart mission
            self.rtl_triggered = False
            self.start_mission()
        else:
            self.get_logger().warning(f'Resume failed: {result.message}')
            # Start fresh mission
            self.start_mission()


def main(args=None):
    rclpy.init(args=args)
    
    node = DemoSimulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simulation stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
