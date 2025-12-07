---
sidebar_position: 4
title: 'Nav2: Navigation for Humanoid Movement'
---

# Nav2: Navigation for Humanoid Movement

This section covers the Navigation 2 (Nav2) system, which provides path planning and navigation capabilities for mobile robots, with specific focus on humanoid robot applications.

## Learning Objectives

After completing this section, you will be able to:
- Configure and deploy Nav2 for humanoid robot navigation
- Implement path planning algorithms suitable for humanoid locomotion
- Set up costmaps for humanoid-specific navigation constraints
- Integrate Nav2 with perception systems for dynamic obstacle avoidance
- Customize Nav2 for humanoid-specific movement patterns

## Introduction to Nav2

Navigation 2 (Nav2) is the navigation stack for ROS 2, providing a complete solution for robot path planning and navigation. For humanoid robots, Nav2 requires special configuration to account for bipedal locomotion, balance constraints, and human-like navigation patterns.

### Nav2 Architecture

Nav2 consists of several key components:
- **Navigation Server**: Centralized navigation executive
- **Planner Server**: Global path planning
- **Controller Server**: Local path following and obstacle avoidance
- **Recovery Server**: Behavior trees for recovery actions
- **BT Navigator**: Behavior tree-based navigation executor
- **Lifecycle Manager**: Component lifecycle management

### Humanoid-Specific Considerations

Humanoid robots have unique navigation requirements:
- **Balance Constraints**: Must maintain center of mass during movement
- **Step Planning**: Requires discrete step placement rather than continuous motion
- **Stability**: Need to ensure stable foot placement at all times
- **Human-like Motion**: Should navigate in a manner similar to human walking

## Nav2 Installation and Setup

### Installing Nav2

```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui

# Install additional packages for humanoid navigation
sudo apt install ros-humble-nav2-rviz-plugins
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-amcl
sudo apt install ros-humble-nav2-lifecycle-manager
```

### Basic Nav2 Launch

```xml
<!-- launch/nav2_humanoid.launch.py -->
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Parameters
    params_file_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    )

    bt_xml_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    )

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'))

    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the Nav2 system'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    ld.add_action(DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=bt_xml_path,
        description='Full path to the behavior tree xml file to use'))

    ld.add_action(DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='false',
        description='Whether to set the map subscriber QoS to transient local'))

    # Lifecycle manager
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]))

    # Map server
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml')}]))  # Example map

    # AMCL (Adaptive Monte Carlo Localization)
    ld.add_action(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]))

    # Planner server
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'planner_server_params_file': params_file}]))

    # Controller server
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'controller_server_params_file': params_file}]))

    # Behavior tree navigator
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'default_bt_xml_filename': default_bt_xml_filename},
                    {'bt_navigator_params_file': params_file}]))

    # Recovery server
    ld.add_action(Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'recoveries_server_params_file': params_file}]))

    # Waypoint follower
    ld.add_action(Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'waypoint_follower_params_file': params_file}]))

    return ld
```

## Humanoid-Specific Configuration

### Nav2 Parameters for Humanoid Robots

```yaml
# config/nav2_humanoid_params.yaml
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
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: False

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: False

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

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: False

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

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

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
  local_costmap_client:
    ros__parameters:
      use_sim_time: False
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

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
  global_costmap_client:
    ros__parameters:
      use_sim_time: False
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

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

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

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

recoveries_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

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

## Humanoid Locomotion Planning

### Step Planning for Bipedal Robots

```python
# Example: Humanoid step planning
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
import numpy as np
from scipy.spatial import KDTree

class HumanoidStepPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_step_planner')

        # Subscribe to global path from Nav2
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        # Publish discrete steps for humanoid
        self.step_pub = self.create_publisher(
            Path,
            '/humanoid_steps',
            10
        )

        # Publish visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/step_markers',
            10
        )

        # Humanoid-specific parameters
        self.step_length = 0.3  # Maximum step length in meters
        self.step_width = 0.2   # Lateral step capability
        self.step_height = 0.1  # Maximum step-up height

    def path_callback(self, msg):
        """Convert continuous path to discrete humanoid steps"""
        if len(msg.poses) < 2:
            return

        # Convert path to steps
        steps = self.convert_path_to_steps(msg.poses)

        # Create discrete step path
        step_path = Path()
        step_path.header = msg.header
        step_path.poses = steps

        # Publish discrete steps
        self.step_pub.publish(step_path)

        # Publish visualization
        self.publish_step_markers(steps)

    def convert_path_to_steps(self, poses):
        """Convert continuous path to discrete humanoid steps"""
        steps = []

        # Start with first pose
        if len(poses) > 0:
            steps.append(poses[0])

        # Process path in segments
        i = 0
        while i < len(poses) - 1:
            current_pose = poses[i].pose.position
            next_pose = poses[i + 1].pose.position

            # Calculate distance to next point
            dist = np.sqrt(
                (next_pose.x - current_pose.x)**2 +
                (next_pose.y - current_pose.y)**2
            )

            if dist <= self.step_length:
                # Add next pose if within step distance
                steps.append(poses[i + 1])
                i += 1
            else:
                # Calculate intermediate step
                step_x = current_pose.x + (next_pose.x - current_pose.x) * self.step_length / dist
                step_y = current_pose.y + (next_pose.y - current_pose.y) * self.step_length / dist

                # Create intermediate step pose
                step_pose = PoseStamped()
                step_pose.header = poses[i].header
                step_pose.pose.position.x = step_x
                step_pose.pose.position.y = step_y
                step_pose.pose.position.z = current_pose.z  # Maintain height

                # Copy orientation from original path
                step_pose.pose.orientation = poses[i].pose.orientation

                steps.append(step_pose)
                # Don't increment i, check distance to next original pose again

        return steps

    def publish_step_markers(self, steps):
        """Publish visualization markers for steps"""
        marker_array = MarkerArray()

        for i, step in enumerate(steps):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "steps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose = step.pose
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
```

### Balance-Aware Path Planning

```python
# Example: Balance-aware path planning
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np

class BalanceAwarePlanner(Node):
    def __init__(self):
        super().__init__('balance_aware_planner')

        # Subscribe to path and sensor data
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publish adjusted path
        self.adjusted_path_pub = self.create_publisher(
            Path,
            '/balance_safe_plan',
            10
        )

        # Robot parameters
        self.foot_separation = 0.2  # Distance between feet
        self.com_height = 0.8       # Center of mass height
        self.support_polygon_margin = 0.1  # Safety margin

        # Store sensor data
        self.scan_data = None

    def path_callback(self, msg):
        """Process path with balance constraints"""
        if not self.scan_data:
            # Publish original path if no sensor data
            self.adjusted_path_pub.publish(msg)
            return

        # Adjust path for balance safety
        adjusted_path = self.adjust_path_for_balance(msg)

        # Publish adjusted path
        self.adjusted_path_pub.publish(adjusted_path)

    def adjust_path_for_balance(self, original_path):
        """Adjust path considering balance constraints"""
        adjusted_path = Path()
        adjusted_path.header = original_path.header

        for pose in original_path.poses:
            # Check if pose is balance-safe
            safe_pose = self.ensure_balance_safety(pose.pose.position, pose.pose.orientation)

            # Create new pose with safe position
            safe_pose_stamped = PoseStamped()
            safe_pose_stamped.header = original_path.header
            safe_pose_stamped.pose.position = safe_pose.position
            safe_pose_stamped.pose.orientation = safe_pose.orientation

            adjusted_path.poses.append(safe_pose_stamped)

        return adjusted_path

    def ensure_balance_safety(self, position, orientation):
        """Ensure position is within balance safety constraints"""
        # Calculate support polygon based on current stance
        # This is a simplified example - real implementation would consider
        # current foot positions and dynamic balance constraints

        # Check distance to obstacles in all directions
        safe_position = Point()
        safe_position.x = position.x
        safe_position.y = position.y
        safe_position.z = position.z

        # Implement balance safety checks here
        # For example, ensure path point is within stable region
        # considering foot placement and center of mass

        return safe_position

    def scan_callback(self, msg):
        """Store laser scan data for obstacle detection"""
        self.scan_data = msg
```

## Integration with Perception Systems

### Dynamic Obstacle Avoidance

```python
# Example: Dynamic obstacle integration with Nav2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial import distance

class DynamicObstacleIntegrator(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_integrator')

        # Subscribe to various sensor inputs
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10
        )

        # Subscribe to robot velocity
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for obstacle status
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/obstacle_detected',
            10
        )

        # Publisher for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/dynamic_obstacles',
            10
        )

        # Track dynamic obstacles
        self.dynamic_obstacles = []
        self.robot_velocity = Twist()
        self.last_positions = {}

    def scan_callback(self, msg):
        """Process laser scan for dynamic obstacle detection"""
        # Convert scan to points in robot frame
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        points = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                x = r * np.cos(angles[i])
                y = r * np.sin(angles[i])
                points.append([x, y])

        # Detect moving obstacles by comparing with previous scans
        self.detect_moving_obstacles(points)

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D obstacle detection"""
        # Process 3D point cloud data for comprehensive obstacle detection
        # This would typically involve PCL or similar processing
        pass

    def cmd_vel_callback(self, msg):
        """Update robot velocity for prediction"""
        self.robot_velocity = msg

    def detect_moving_obstacles(self, current_points):
        """Detect moving obstacles by comparing with previous positions"""
        # Simple approach: compare current scan with previous
        if not hasattr(self, 'previous_points'):
            self.previous_points = current_points
            return

        # Calculate motion vectors for potential obstacles
        for i, current_point in enumerate(current_points):
            if i < len(self.previous_points):
                prev_point = self.previous_points[i]

                # Calculate displacement
                displacement = np.array(current_point) - np.array(prev_point)
                distance_moved = np.linalg.norm(displacement)

                # If moved significantly, consider as dynamic obstacle
                if distance_moved > 0.1:  # Threshold for movement
                    # Update obstacle tracking
                    obstacle_id = i
                    self.update_dynamic_obstacle(obstacle_id, current_point, displacement)

        self.previous_points = current_points

    def update_dynamic_obstacle(self, obstacle_id, position, velocity):
        """Update tracked dynamic obstacle"""
        # Store or update dynamic obstacle information
        obstacle_info = {
            'id': obstacle_id,
            'position': position,
            'velocity': velocity,
            'timestamp': self.get_clock().now()
        }

        # Update or add to tracked obstacles
        for i, obs in enumerate(self.dynamic_obstacles):
            if obs['id'] == obstacle_id:
                self.dynamic_obstacles[i] = obstacle_info
                return

        # Add new obstacle if not tracked
        self.dynamic_obstacles.append(obstacle_info)

    def predict_obstacle_trajectory(self, obstacle_info, time_horizon=2.0):
        """Predict obstacle trajectory for collision avoidance"""
        # Simple constant velocity prediction
        predicted_positions = []
        dt = 0.1  # Time step

        for t in np.arange(0, time_horizon, dt):
            predicted_x = obstacle_info['position'][0] + obstacle_info['velocity'][0] * t
            predicted_y = obstacle_info['position'][1] + obstacle_info['velocity'][1] * t
            predicted_positions.append([predicted_x, predicted_y])

        return predicted_positions
```

## Custom Controllers for Humanoid Movement

### Humanoid-Specific Controller

```python
# Example: Custom humanoid controller
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/humanoid_steps',  # From step planner
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.step_status_pub = self.create_publisher(
            Float32,
            '/step_progress',
            10
        )

        # Controller parameters
        self.linear_vel_limit = 0.3  # m/s
        self.angular_vel_limit = 0.5  # rad/s
        self.position_tolerance = 0.1  # m
        self.angle_tolerance = 0.1    # rad

        # Controller state
        self.current_path = []
        self.current_index = 0
        self.current_pose = None
        self.controller_active = False

    def path_callback(self, msg):
        """Receive path and start following"""
        if len(msg.poses) > 0:
            self.current_path = msg.poses
            self.current_index = 0
            self.controller_active = True
            self.get_logger().info(f"Received path with {len(msg.poses)} steps")

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose

        if self.controller_active and self.current_pose:
            self.follow_path()

    def follow_path(self):
        """Follow the current path with humanoid-specific constraints"""
        if self.current_index >= len(self.current_path):
            # Path completed
            self.stop_robot()
            self.controller_active = False
            return

        target_pose = self.current_path[self.current_index].pose

        # Calculate error to target
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance_to_target = np.sqrt(dx*dx + dy*dy)

        # Calculate target angle
        target_angle = np.arctan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate angle error
        angle_error = self.normalize_angle(target_angle - current_yaw)

        # Generate velocity commands with humanoid constraints
        cmd_vel = Twist()

        if distance_to_target > self.position_tolerance:
            # Move toward target with limited velocity
            cmd_vel.linear.x = min(self.linear_vel_limit * 0.7,
                                  max(0.05, distance_to_target * 0.5))

            # Adjust orientation if significantly off
            if abs(angle_error) > self.angle_tolerance:
                cmd_vel.angular.z = max(-self.angular_vel_limit,
                                       min(self.angular_vel_limit,
                                           angle_error * 1.0))
        else:
            # Reached current target, move to next
            self.current_index += 1
            if self.current_index >= len(self.current_path):
                self.get_logger().info("Path completed")
                self.stop_robot()
                self.controller_active = False
                return

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish progress
        progress = Float32()
        progress.data = float(self.current_index) / len(self.current_path)
        self.step_status_pub.publish(progress)

    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
```

## Lab Activity: Humanoid Navigation System

Create a complete humanoid navigation system with:
1. Nav2 configured for humanoid constraints
2. Step planning for bipedal locomotion
3. Balance-aware path adjustment
4. Dynamic obstacle integration
5. Humanoid-specific controller

### Steps:
1. Install and configure Nav2 for humanoid navigation
2. Implement step planning algorithm
3. Add balance-aware path constraints
4. Integrate dynamic obstacle detection
5. Create humanoid-specific controller
6. Test system in simulation with various scenarios

### Expected Outcome:
- Functional Nav2 system adapted for humanoid robots
- Step planning that considers bipedal constraints
- Balance-aware navigation avoiding stability issues
- Dynamic obstacle avoidance working
- Humanoid-specific controller following paths safely

## Best Practices

1. **Safety First**: Always implement safety checks for humanoid balance
2. **Parameter Tuning**: Carefully tune Nav2 parameters for humanoid characteristics
3. **Step Planning**: Consider discrete step placement for bipedal robots
4. **Balance Constraints**: Integrate balance considerations into navigation
5. **Testing**: Test navigation system in various scenarios before deployment
6. **Fallback Plans**: Implement recovery behaviors for navigation failures

## Checklist

- [ ] Nav2 installation and configuration for humanoid robots
- [ ] Step planning algorithm implemented
- [ ] Balance-aware path adjustment configured
- [ ] Dynamic obstacle integration working
- [ ] Humanoid-specific controller created
- [ ] System tested with various scenarios
- [ ] Lab activity completed successfully

## Next Steps

In the next section, we'll explore the Sim-to-Real workflow, which focuses on transferring learned behaviors and models from simulation to real hardware.