---
sidebar_position: 3
title: 'Navigation Planning'
---

# Navigation Planning

This section covers the implementation of navigation planning for the capstone project, enabling the humanoid robot to navigate to specified locations and plan paths around obstacles.

## Learning Objectives

After completing this section, you will be able to:
- Implement navigation planning using Nav2
- Configure navigation parameters for humanoid robots
- Plan paths in complex environments
- Handle navigation failures and recoveries
- Integrate navigation with voice command processing

## Navigation Architecture

The navigation system uses the Navigation 2 (Nav2) framework, adapted for humanoid robot characteristics:

1. **Global Planner**: Plans the overall path from start to goal
2. **Local Planner**: Handles immediate obstacle avoidance and path following
3. **Controller**: Executes the planned path with robot-specific control
4. **Recovery Behaviors**: Handles navigation failures and stuck situations
5. **Map Management**: Manages occupancy maps and localization

## Nav2 Configuration for Humanoid Robots

### Navigation Parameters

```yaml
# config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"  # Changed for humanoid
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: False
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_footprint  # Changed for humanoid
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0  # Lower frequency for humanoid stability
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.1
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"  # Or use DwbController
      debug: false
      time_steps: 25
      control_horizon: 3
      nonholonomic: true
      model_type: "StatePropagator"
      xy_goal_tolerance: 0.25  # Larger tolerance for humanoid
      yaw_goal_tolerance: 0.25
      state_propagator: "HolonomicPropagator"
      collision_penalty: 100.0
      goal_face_point_distance: 0.2
      goal_front_tolerance: 0.1
      goal_angle_tolerance: 0.2
      trajectory_visualization_plugin: "nav2_trajectory_visualization::TrajectoryVisualization"
      transform_tolerance: 0.1
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      progress_checker: "progress_checker"
      goal_checker: "goal_checker"

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint  # Changed for humanoid
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for precise foot placement
      robot_radius: 0.3  # Larger radius for humanoid safety
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint  # Changed for humanoid
      use_sim_time: False
      robot_radius: 0.3  # Larger radius for humanoid
      resolution: 0.05  # Higher resolution
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # Larger tolerance for humanoid
      use_astar: false
      allow_unknown: true

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_granularity: 0.017
      angle: 1.57
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_granularity: 0.0044
      duration: 1.0
      sim_time: 1.0
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_time: 1.0
      duration: 5.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

## Navigation Node Implementation

### Navigation Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from typing import Optional
import json

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers for commands
        self.command_sub = self.create_subscription(
            String, '/processed_command', self.command_callback, 10
        )

        # Publishers for status
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)

        # Wait for navigation server
        self.nav_client.wait_for_server()
        self.get_logger().info("Navigation Controller initialized")

    def command_callback(self, msg):
        """Process navigation commands from voice command system"""
        try:
            command_data = json.loads(msg.data)

            if command_data.get('action_type') == 'navigation':
                target_location = command_data.get('target_location')

                if target_location:
                    # Look up the coordinates for the location
                    location_coords = self.lookup_location_coordinates(target_location)

                    if location_coords:
                        self.navigate_to_location(location_coords)
                    else:
                        self.get_logger().error(f"Unknown location: {target_location}")
                else:
                    self.get_logger().error("No target location specified in command")

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in command message")
        except Exception as e:
            self.get_logger().error(f"Error processing navigation command: {e}")

    def lookup_location_coordinates(self, location_name: str) -> Optional[dict]:
        """Look up coordinates for a named location"""
        # In practice, this would be a database or parameter lookup
        location_map = {
            'kitchen': {'x': 5.0, 'y': 3.0, 'theta': 0.0},
            'living_room': {'x': 2.0, 'y': 1.0, 'theta': 0.0},
            'bedroom': {'x': -1.0, 'y': 4.0, 'theta': 1.57},
            'office': {'x': 3.0, 'y': -2.0, 'theta': 3.14},
            'entrance': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
        }

        return location_map.get(location_name.lower())

    def navigate_to_location(self, location_coords: dict):
        """Navigate to specified coordinates"""
        goal_msg = NavigateToPose.Goal()

        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = location_coords['x']
        goal_msg.pose.pose.position.y = location_coords['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        import math
        theta = location_coords['theta']
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send navigation goal
        self.get_logger().info(f"Sending navigation goal to ({location_coords['x']}, {location_coords['y']})")

        # Publish status
        status_msg = String()
        status_msg.data = f"Navigating to {location_coords}"
        self.status_pub.publish(status_msg)

        # Send goal and wait for result
        goal_handle = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        goal_handle.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Process feedback as needed
        self.get_logger().debug(f"Navigation feedback: {feedback}")

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        goal_handle = future.result()
        result = goal_handle.get_result()

        if result.result:
            self.get_logger().info("Navigation completed successfully")
            status_msg = String()
            status_msg.data = "Navigation completed successfully"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error("Navigation failed")
            status_msg = String()
            status_msg.data = "Navigation failed"
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Navigation Controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Navigation Considerations

### Balance-Aware Path Planning

```python
import numpy as np
from scipy.spatial import distance
from geometry_msgs.msg import Point

class BalanceAwareNavigator:
    def __init__(self, robot_params):
        self.foot_separation = robot_params.get('foot_separation', 0.2)  # Distance between feet
        self.com_height = robot_params.get('com_height', 0.8)  # Center of mass height
        self.max_step_length = robot_params.get('max_step_length', 0.3)  # Maximum step length
        self.max_step_width = robot_params.get('max_step_width', 0.2)  # Maximum lateral step
        self.support_polygon_margin = robot_params.get('support_polygon_margin', 0.1)  # Safety margin

    def plan_balance_safe_path(self, start_pose, goal_pose, obstacles=None):
        """Plan a path that maintains robot balance"""
        # Basic path planning with balance constraints
        path = self.basic_path_planning(start_pose, goal_pose)

        # Apply balance constraints
        balance_safe_path = self.apply_balance_constraints(path, obstacles)

        return balance_safe_path

    def apply_balance_constraints(self, path, obstacles=None):
        """Apply balance constraints to a path"""
        if not path:
            return path

        balance_safe_path = [path[0]]  # Start with the first point

        for i in range(1, len(path)):
            current_point = path[i]
            previous_point = balance_safe_path[-1]

            # Check if the step is balance-safe
            if self.is_step_balance_safe(previous_point, current_point, obstacles):
                balance_safe_path.append(current_point)
            else:
                # Find an alternative path that's balance-safe
                alternative_point = self.find_balance_safe_alternative(
                    previous_point, current_point, obstacles
                )
                if alternative_point:
                    balance_safe_path.append(alternative_point)

        return balance_safe_path

    def is_step_balance_safe(self, start_point, end_point, obstacles=None):
        """Check if a step is balance-safe"""
        # Calculate step length
        step_vector = np.array([end_point.x - start_point.x, end_point.y - start_point.y])
        step_length = np.linalg.norm(step_vector)

        # Check step length constraint
        if step_length > self.max_step_length:
            return False

        # Check for obstacles in the path
        if obstacles and self.path_has_obstacles(start_point, end_point, obstacles):
            return False

        # Check balance constraints (simplified)
        # In reality, this would involve more complex balance calculations
        # considering the robot's current state, planned steps, and support polygon
        return True

    def find_balance_safe_alternative(self, start_point, goal_point, obstacles=None):
        """Find a balance-safe alternative to reach the goal"""
        # Try to find intermediate points that are balance-safe
        direction = np.array([goal_point.x - start_point.x, goal_point.y - start_point.y])
        direction_norm = np.linalg.norm(direction)

        if direction_norm == 0:
            return None

        unit_direction = direction / direction_norm

        # Try smaller steps toward the goal
        for step_scale in [0.5, 0.75, 0.25]:
            intermediate = Point()
            intermediate.x = start_point.x + unit_direction[0] * step_scale * self.max_step_length
            intermediate.y = start_point.y + unit_direction[1] * step_scale * self.max_step_length
            intermediate.z = start_point.z  # Maintain height

            if self.is_step_balance_safe(start_point, intermediate, obstacles):
                return intermediate

        return None

    def path_has_obstacles(self, start_point, end_point, obstacles):
        """Check if the straight-line path has obstacles"""
        if not obstacles:
            return False

        # Simple line-of-sight check
        path_vector = np.array([end_point.x - start_point.x, end_point.y - start_point.y])
        path_length = np.linalg.norm(path_vector)

        if path_length == 0:
            return False

        unit_path = path_vector / path_length

        # Check every 0.1m along the path
        step_size = 0.1
        num_steps = int(path_length / step_size) + 1

        for i in range(num_steps + 1):
            check_point = np.array([start_point.x, start_point.y]) + unit_path * min(i * step_size, path_length)

            for obstacle in obstacles:
                obs_pos = np.array([obstacle.position.x, obstacle.position.y])
                distance_to_obs = np.linalg.norm(check_point - obs_pos)

                # Consider obstacle radius (simplified as 0.3m for all obstacles)
                if distance_to_obs < 0.3:
                    return True

        return False

    def basic_path_planning(self, start_pose, goal_pose):
        """Basic path planning (in practice, this would use A*, Dijkstra, etc.)"""
        # For simplicity, return a direct path
        # In practice, use proper path planning algorithms
        path = []

        # Create a straight line path with intermediate waypoints
        num_waypoints = 10
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            point = Point()
            point.x = start_pose.position.x + t * (goal_pose.position.x - start_pose.position.x)
            point.y = start_pose.position.y + t * (goal_pose.position.y - start_pose.position.y)
            point.z = start_pose.position.z
            path.append(point)

        return path
```

## Integration with Voice Command System

### Voice Command to Navigation Mapping

```python
class VoiceNavigationMapper:
    def __init__(self):
        self.location_synonyms = {
            'kitchen': ['kitchen', 'cooking area', 'food area'],
            'living_room': ['living room', 'sitting room', 'lounge', 'family room'],
            'bedroom': ['bedroom', 'sleeping room', 'bedroom area'],
            'office': ['office', 'study', 'work room', 'workspace'],
            'dining_room': ['dining room', 'dining area', 'eat room'],
            'bathroom': ['bathroom', 'restroom', 'washroom'],
            'entrance': ['entrance', 'entry', 'front door', 'main door'],
            'exit': ['exit', 'way out', 'back door'],
        }

    def extract_navigation_intent(self, command_text: str) -> dict:
        """Extract navigation intent from voice command"""
        command_lower = command_text.lower()

        # Look for navigation keywords
        navigation_keywords = [
            'go to', 'navigate to', 'move to', 'walk to', 'travel to',
            'reach', 'get to', 'head to', 'go towards', 'move towards'
        ]

        for keyword in navigation_keywords:
            if keyword in command_lower:
                # Extract the location part
                location_part = command_lower.split(keyword)[1].strip()

                # Clean up the location string
                location = self.clean_location_string(location_part)

                # Find the canonical location name
                canonical_location = self.find_canonical_location(location)

                if canonical_location:
                    return {
                        'action_type': 'navigation',
                        'target_location': canonical_location,
                        'original_command': command_text,
                        'extracted_location': location
                    }

        # If no navigation intent found, return None
        return None

    def clean_location_string(self, location_string: str) -> str:
        """Clean up the extracted location string"""
        # Remove common phrases
        phrases_to_remove = [
            'the ', 'please ', 'can you ', 'would you ', 'for me',
            'now', 'right', 'straight', 'directly', 'immediately'
        ]

        cleaned = location_string.strip()

        for phrase in phrases_to_remove:
            cleaned = cleaned.replace(phrase, '').strip()

        # Remove punctuation
        import string
        cleaned = cleaned.translate(str.maketrans('', '', string.punctuation))

        return cleaned.strip()

    def find_canonical_location(self, location_guess: str) -> str:
        """Find the canonical location name from a guess"""
        for canonical_name, synonyms in self.location_synonyms.items():
            if location_guess in synonyms or canonical_name == location_guess:
                return canonical_name

        # If no exact match, try fuzzy matching
        return self.fuzzy_match_location(location_guess)

    def fuzzy_match_location(self, location_guess: str) -> str:
        """Use fuzzy matching to find the closest location"""
        from difflib import get_close_matches

        all_synonyms = []
        synonym_to_canonical = {}

        for canonical, synonyms in self.location_synonyms.items():
            all_synonyms.extend(synonyms)
            for syn in synonyms:
                synonym_to_canonical[syn] = canonical

        matches = get_close_matches(location_guess, all_synonyms, n=1, cutoff=0.6)

        if matches:
            return synonym_to_canonical[matches[0]]

        return None  # No match found
```

## Testing and Validation

### Navigation Testing Framework

```python
import unittest
import numpy as np
from geometry_msgs.msg import Point, Pose

class TestNavigation(unittest.TestCase):
    def setUp(self):
        self.robot_params = {
            'foot_separation': 0.2,
            'com_height': 0.8,
            'max_step_length': 0.3,
            'max_step_width': 0.2,
            'support_polygon_margin': 0.1
        }
        self.navigator = BalanceAwareNavigator(self.robot_params)
        self.voice_mapper = VoiceNavigationMapper()

    def test_basic_path_planning(self):
        """Test basic path planning functionality"""
        start_pose = Pose()
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0

        goal_pose = Pose()
        goal_pose.position.x = 5.0
        goal_pose.position.y = 5.0

        path = self.navigator.basic_path_planning(start_pose, goal_pose)
        self.assertGreater(len(path), 0)
        self.assertEqual(path[0].x, 0.0)
        self.assertEqual(path[0].y, 0.0)

    def test_balance_constraint_application(self):
        """Test that balance constraints are applied properly"""
        path = [Point(x=0.0, y=0.0, z=0.0), Point(x=10.0, y=0.0, z=0.0)]  # Too far for single step

        balanced_path = self.navigator.apply_balance_constraints(path)

        # The balanced path should have more intermediate points
        self.assertGreater(len(balanced_path), 2)

    def test_voice_command_parsing(self):
        """Test voice command parsing"""
        test_commands = [
            ("Please go to the kitchen", "kitchen"),
            ("Navigate to the living room", "living_room"),
            ("Move to the bedroom", "bedroom"),
        ]

        for command, expected_location in test_commands:
            result = self.voice_mapper.extract_navigation_intent(command)
            if result:
                self.assertEqual(result['target_location'], expected_location)
            else:
                self.fail(f"Failed to parse command: {command}")

if __name__ == '__main__':
    unittest.main()
```

## Best Practices

1. **Safety First**: Always implement safety checks for navigation
2. **Parameter Tuning**: Carefully tune navigation parameters for humanoid characteristics
3. **Step Planning**: Consider discrete step placement for bipedal robots
4. **Balance Constraints**: Integrate balance considerations into navigation
5. **Testing**: Test navigation system in various scenarios before deployment
6. **Fallback Plans**: Implement recovery behaviors for navigation failures

## Integration with Capstone System

The navigation planning system integrates with the overall capstone project by:
- Receiving navigation commands from the voice command processing system
- Planning safe paths that consider humanoid balance constraints
- Executing navigation while monitoring for obstacles and failures
- Providing feedback to the overall task management system

This navigation planning system enables the humanoid robot to move safely and effectively through the environment as part of the capstone project, supporting the overall goal of autonomous task execution based on voice commands.