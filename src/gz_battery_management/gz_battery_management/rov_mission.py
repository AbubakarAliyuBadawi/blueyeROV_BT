# rov_mission.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from mundus_mir_msgs.srv import AddWaypoint, RunWaypointController, GoToWaypoints, GetWaypointStatus, ClearWaypoints
from mundus_mir_msgs.msg import BatteryStatus
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from mundus_mir_msgs.msg import ReturnRecommendation

class BatteryMonitor:
   def __init__(self, node):
       self.battery_level = 100.0
       self.should_return = False
       self.battery_sub = node.create_subscription(
           BatteryStatus,
           '/blueye/battery',
           self._battery_callback,
           10)
       # Add return recommendation subscription
       self.return_sub = node.create_subscription(
           ReturnRecommendation,
           '/blueye/return_recommendation',
           self._return_callback,
           10)

   def _battery_callback(self, msg):
       self.battery_level = msg.state_of_charge * 100
       
   def _return_callback(self, msg):
       """Handle return recommendations"""
       self.should_return = msg.should_return 
       
   def is_low(self):
       return self.should_return

class UndockState(State):
    def __init__(self, node: Node) -> None:
        super().__init__(outcomes=['undocked', 'running', 'failed'])
        self.node = node
        self.add_client = node.create_client(AddWaypoint, '/blueye/add_waypoint')
        self.run_client = node.create_client(RunWaypointController, '/blueye/run_waypoint_controller')
        self.go_client = node.create_client(GoToWaypoints, '/blueye/go_to_waypoints')
        self.status_client = node.create_client(GetWaypointStatus, '/blueye/get_waypoint_status')
        self.clear_client = node.create_client(ClearWaypoints, '/blueye/clear_waypoints')
        self.first_run = True

    def execute(self, blackboard: Blackboard) -> str:
        if self.first_run:
            # First clear any existing waypoints
            clear_req = ClearWaypoints.Request()
            if not self.clear_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.clear_client.call_async(clear_req)
            rclpy.spin_until_future_complete(self.node, future)

            # Start the waypoint controller first
            run_req = RunWaypointController.Request()
            run_req.run = True
            if not self.run_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.run_client.call_async(run_req)
            rclpy.spin_until_future_complete(self.node, future)

            # Add the waypoint
            add_req = AddWaypoint.Request()
            add_req.x = float(-9.5)
            add_req.y = float(8.54)
            add_req.z = float(95.7)
            add_req.desired_velocity = float(0.3)  # Slower speed for safety
            add_req.fixed_heading = False
            add_req.heading = float(0.0)

            if not self.add_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.add_client.call_async(add_req)
            rclpy.spin_until_future_complete(self.node, future)

            # Start moving to waypoints
            go_req = GoToWaypoints.Request()
            go_req.run = True
            if not self.go_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.go_client.call_async(go_req)
            rclpy.spin_until_future_complete(self.node, future)
            
            self.first_run = False
            return 'running'

        status_req = GetWaypointStatus.Request()
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            return 'failed'
        future = self.status_client.call_async(status_req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        if "Currently going to waypoint: 0" in result.status_code:
            return 'undocked'
        return 'running'

class InspectPipelineState(State):
   def __init__(self, node: Node) -> None:
       super().__init__(outcomes=['completed', 'running', 'low_battery', 'failed'])
       self.node = node
       self.current_point = 0
       self.waypoints = [(-18, -25, 90), (20, 55, 90)]
       self.add_client = node.create_client(AddWaypoint, '/blueye/add_waypoint')
       self.run_client = node.create_client(RunWaypointController, '/blueye/run_waypoint_controller') 
       self.go_client = node.create_client(GoToWaypoints, '/blueye/go_to_waypoints')
       self.status_client = node.create_client(GetWaypointStatus, '/blueye/get_waypoint_status')
       self.clear_client = node.create_client(ClearWaypoints, '/blueye/clear_waypoints')
       self.first_run = True

   def execute(self, blackboard: Blackboard) -> str:
       battery_monitor = blackboard["battery_monitor"]
       if battery_monitor.is_low():
           return 'low_battery'

       if self.first_run:
           clear_req = ClearWaypoints.Request()
           if not self.clear_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.clear_client.call_async(clear_req)
           rclpy.spin_until_future_complete(self.node, future)

           wp = self.waypoints[self.current_point]
           add_req = AddWaypoint.Request()
           add_req.x, add_req.y, add_req.z = float(wp[0]), float(wp[1]), float(wp[2])
           add_req.desired_velocity = float(0.5)
           add_req.fixed_heading = False
           add_req.heading = float(0.0)

           if not self.add_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.add_client.call_async(add_req)
           rclpy.spin_until_future_complete(self.node, future)

           run_req = RunWaypointController.Request()
           run_req.run = True
           if not self.run_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.run_client.call_async(run_req)
           rclpy.spin_until_future_complete(self.node, future)

           go_req = GoToWaypoints.Request()
           go_req.run = True
           if not self.go_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.go_client.call_async(go_req)
           rclpy.spin_until_future_complete(self.node, future)
           
           self.first_run = False
           return 'running'

       status_req = GetWaypointStatus.Request()
       if not self.status_client.wait_for_service(timeout_sec=1.0):
           return 'failed'
       future = self.status_client.call_async(status_req)
       rclpy.spin_until_future_complete(self.node, future)
       result = future.result()

       if "Currently going to waypoint: 0" in result.status_code:
           self.current_point += 1
           if self.current_point >= len(self.waypoints):
               return 'completed'
           
           clear_req = ClearWaypoints.Request()
           if not self.clear_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.clear_client.call_async(clear_req)
           rclpy.spin_until_future_complete(self.node, future)

           wp = self.waypoints[self.current_point]
           add_req = AddWaypoint.Request()
           add_req.x, add_req.y, add_req.z = float(wp[0]), float(wp[1]), float(wp[2])
           add_req.desired_velocity = float(0.5)
           add_req.fixed_heading = False
           add_req.heading = float(0.0)

           if not self.add_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.add_client.call_async(add_req)
           rclpy.spin_until_future_complete(self.node, future)

           go_req = GoToWaypoints.Request()
           go_req.run = True
           if not self.go_client.wait_for_service(timeout_sec=1.0):
               return 'failed'
           future = self.go_client.call_async(go_req)
           rclpy.spin_until_future_complete(self.node, future)

       return 'running'

class ReturnToDockState(State):
    def __init__(self, node: Node) -> None:
        super().__init__(outcomes=['docked', 'running', 'failed'])
        self.node = node
        self.add_client = node.create_client(AddWaypoint, '/blueye/add_waypoint')
        self.run_client = node.create_client(RunWaypointController, '/blueye/run_waypoint_controller')
        self.go_client = node.create_client(GoToWaypoints, '/blueye/go_to_waypoints')
        self.status_client = node.create_client(GetWaypointStatus, '/blueye/get_waypoint_status')
        self.clear_client = node.create_client(ClearWaypoints, '/blueye/clear_waypoints')
        self.first_run = True
        
        # Add position tracking
        self.current_position = None
        self.dock_position = [-8.5, 8.54, 95.3]  # Store dock position
        self.position_sub = node.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self._position_callback,
            10
        )
        
    def _position_callback(self, msg):
        """Store current position from odometry"""
        self.current_position = msg.pose.pose.position
        
    def is_at_dock(self):
        """Check if ROV is physically at dock position"""
        if self.current_position is None:
            return False
            
        # Calculate distance to dock
        distance = ((self.current_position.x - self.dock_position[0])**2 + 
                   (self.current_position.y - self.dock_position[1])**2 + 
                   (self.current_position.z - self.dock_position[2])**2)**0.5
                   
        # Consider docked if within 0.1 meters of dock position
        return distance < 0.1

    def execute(self, blackboard: Blackboard) -> str:
        battery_monitor = blackboard["battery_monitor"]
        if battery_monitor.is_low() and not self.first_run:
            # Increase speed when battery is low
            self.update_waypoint_speed(1.0)

        if self.first_run:
            # First clear any existing waypoints
            clear_req = ClearWaypoints.Request()
            if not self.clear_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.clear_client.call_async(clear_req)
            rclpy.spin_until_future_complete(self.node, future)

            # Add waypoint for dock
            add_req = AddWaypoint.Request()
            add_req.x = float(self.dock_position[0])
            add_req.y = float(self.dock_position[1])
            add_req.z = float(self.dock_position[2])
            add_req.desired_velocity = float(0.5 if not battery_monitor.is_low() else 1.0)
            add_req.fixed_heading = False
            add_req.heading = float(0.0)

            if not self.add_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.add_client.call_async(add_req)
            rclpy.spin_until_future_complete(self.node, future)

            # Start moving to waypoint
            go_req = GoToWaypoints.Request()
            go_req.run = True
            if not self.go_client.wait_for_service(timeout_sec=1.0):
                return 'failed'
            future = self.go_client.call_async(go_req)
            rclpy.spin_until_future_complete(self.node, future)
            
            self.first_run = False
            return 'running'

        # Check waypoint status
        status_req = GetWaypointStatus.Request()
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            return 'failed'
        future = self.status_client.call_async(status_req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        # Only return docked if we're both at waypoint AND physically at dock
        if "Currently going to waypoint: 0" in result.status_code and self.is_at_dock():
            self.node.get_logger().info('ROV has reached dock position')
            return 'docked'
            
        return 'running'

    def update_waypoint_speed(self, speed):
        """Update the waypoint with new speed"""
        clear_req = ClearWaypoints.Request()
        if not self.clear_client.wait_for_service(timeout_sec=1.0):
            return
        self.clear_client.call_async(clear_req)

        add_req = AddWaypoint.Request()
        add_req.x = float(self.dock_position[0])
        add_req.y = float(self.dock_position[1])
        add_req.z = float(self.dock_position[2])
        add_req.desired_velocity = float(speed)
        add_req.fixed_heading = False
        add_req.heading = float(0.0)
        
        if not self.add_client.wait_for_service(timeout_sec=1.0):
            return
        self.add_client.call_async(add_req)

        go_req = GoToWaypoints.Request()
        go_req.run = True
        if not self.go_client.wait_for_service(timeout_sec=1.0):
            return
        self.go_client.call_async(go_req)

def main():
    rclpy.init()
    node = rclpy.create_node('rov_mission')
    set_ros_loggers()

    battery_monitor = BatteryMonitor(node)
    blackboard = Blackboard()
    blackboard["battery_monitor"] = battery_monitor

    sm = StateMachine(outcomes=['mission_complete', 'mission_failed'])

    # Add the new undock state first
    sm.add_state('UNDOCK',
                UndockState(node),
                {'undocked': 'INSPECT_PIPELINE',
                 'running': 'UNDOCK',
                 'failed': 'mission_failed'})

    sm.add_state('INSPECT_PIPELINE',
                InspectPipelineState(node),
                {'completed': 'RETURN_TO_DOCK',
                 'low_battery': 'RETURN_TO_DOCK',
                 'running': 'INSPECT_PIPELINE',
                 'failed': 'mission_failed'})

    sm.add_state('RETURN_TO_DOCK',
                ReturnToDockState(node),
                {'docked': 'mission_complete',
                 'running': 'RETURN_TO_DOCK',
                 'failed': 'mission_failed'})

    viewer = YasminViewerPub("ROV Pipeline Inspection", sm)

    try:
        outcome = sm.execute(blackboard)
        node.get_logger().info(f'Mission outcome: {outcome}')
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        rclpy.shutdown()