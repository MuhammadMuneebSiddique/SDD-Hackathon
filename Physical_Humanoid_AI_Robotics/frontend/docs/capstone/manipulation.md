---
sidebar_position: 5
title: 'Manipulation Planning'
---

# Manipulation Planning

This section covers the implementation of manipulation planning for the capstone project, enabling the humanoid robot to interact with objects in its environment through grasping, transporting, and placing actions.

## Learning Objectives

After completing this section, you will be able to:
- Implement manipulation planning for humanoid robots
- Integrate perception with manipulation for object interaction
- Plan and execute grasping and placement actions
- Handle manipulation failures and recovery
- Coordinate manipulation with navigation and voice commands

## Manipulation Architecture

The manipulation system uses MoveIt2 for motion planning combined with perception data for object interaction:

1. **Perception Integration**: Object detection and pose estimation
2. **Grasp Planning**: Determining appropriate grasp poses
3. **Motion Planning**: Planning collision-free trajectories
4. **Execution**: Executing manipulation actions
5. **Feedback**: Monitoring and error handling

## MoveIt2 Configuration for Humanoid Robot

### Robot Configuration

```yaml
# config/humanoid_moveit_config/config/humanoid.srdf
<?xml version="1.0" encoding="UTF-8"?>
<!--This does not replace URDF, and is not an extension of URDF.
    This is a format for representing semantic information about the robot structure.
    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
-->
<robot name="humanoid">
    <!--GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc-->
    <!--LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included-->
    <!--JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included-->
    <!--CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group-->
    <!--SUBGROUPS: Groups can also be formed by referencing to already defined group names-->
    <group name="right_arm">
        <chain base_link="torso" tip_link="right_hand"/>
    </group>
    <group name="left_arm">
        <chain base_link="torso" tip_link="left_hand"/>
    </group>
    <group name="both_arms">
        <group name="right_arm"/>
        <group name="left_arm"/>
    </group>
    <group name="right_hand">
        <joint name="right_gripper_joint"/>
    </group>
    <group name="left_hand">
        <joint name="left_gripper_joint"/>
    </group>

    <!--GROUP STATES: Purpose: Define a named state for a particular group, in terms of joint values. This is useful to define states like 'folded arms'-->
    <group_state name="right_arm_ready" group="right_arm">
        <joint name="right_shoulder_pitch_joint" value="0"/>
        <joint name="right_shoulder_roll_joint" value="0"/>
        <joint name="right_elbow_joint" value="0"/>
        <joint name="right_wrist_yaw_joint" value="0"/>
        <joint name="right_wrist_pitch_joint" value="0"/>
    </group_state>
    <group_state name="left_arm_ready" group="left_arm">
        <joint name="left_shoulder_pitch_joint" value="0"/>
        <joint name="left_shoulder_roll_joint" value="0"/>
        <joint name="left_elbow_joint" value="0"/>
        <joint name="left_wrist_yaw_joint" value="0"/>
        <joint name="left_wrist_pitch_joint" value="0"/>
    </group_state>

    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="right_hand_eef" parent_link="right_hand" group="right_hand" parent_group="right_arm"/>
    <end_effector name="left_hand_eef" parent_link="left_hand" group="left_hand" parent_group="left_arm"/>

    <!--VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of reference (considered fixed with respect to the robot)-->
    <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>

    <!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. -->
    <disable_collisions link1="base_link" link2="torso" reason="Adjacent"/>
    <disable_collisions link1="torso" link2="head" reason="Adjacent"/>
    <disable_collisions link1="torso" link2="right_shoulder" reason="Adjacent"/>
    <disable_collisions link1="torso" link2="left_shoulder" reason="Adjacent"/>
    <!-- Additional collision pairs would be specified here -->
</robot>
```

### Kinematics Configuration

```yaml
# config/humanoid_moveit_config/config/kinematics.yaml
right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3

left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3

both_arms:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1
  kinematics_solver_attempts: 3
```

### Planning Pipeline Configuration

```yaml
# config/humanoid_moveit_config/config/ompl_planning.yaml
planner_configs:
  SBLkConfigDefault:
    type: geometric::SBL
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
  ESTkConfigDefault:
    type: geometric::EST
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
  LBKPIECEkConfigDefault:
    type: geometric::LBKPIECE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    border_fraction: 0.9  # Fraction of time focused on boarder default: 0.9
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction. default: 0.5
  BKPIECEkConfigDefault:
    type: geometric::BKPIECE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    border_fraction: 0.9  # Fraction of time focused on boarder default: 0.9
    failed_expansion_score_factor: 0.5  # When extending motion fails, scale score by factor. default: 0.5
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction. default: 0.5
  KPIECEkConfigDefault:
    type: geometric::KPIECE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
    border_fraction: 0.9  # Fraction of time focused on boarder default: 0.9
    failed_expansion_score_factor: 0.5  # When extending motion fails, scale score by factor. default: 0.5
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction. default: 0.5
  RRTkConfigDefault:
    type: geometric::RRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
  RRTstarkConfigDefault:
    type: geometric::RRTstar
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
    delay_collision_checking: 1  # Stop collision checking as soon as C-free parent found. default: 1
  TRRTkConfigDefault:
    type: geometric::TRRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
    max_states_failed: 10  # when to start increasing temp. default: 10
    temp_change_factor: 2.0  # how much to increase or decrease temp. default: 2.0
    min_temperature: 10e-10  # lower limit of temp change. default: 10e-10
    init_temperature: 10e-6  # initial temperature. default: 10e-6
    frountier_threshold: 0.0  # dist new state to nearest neighbor to disqualify as frontier. default: 0.0 set in setup()
    frountierNodeRatio: 0.1  # 1/10, or 1 non-frontier for every 10 frontier. default: 0.1
  PRMkConfigDefault:
    type: geometric::PRM
    max_nearest_neighbors: 10  # use k nearest neighbors. default: 10
  PRMstarkConfigDefault:
    type: geometric::PRMstar
  FMTkConfigDefault:
    type: geometric::FMT
    num_samples: 1000  # number of states that will be sampled. default: 1000
    radius_multiplier: 1.1  # multiplier for connection radius. default: 1.1
    nearest_k: 1  # use Knearest strategy. default: 1
    cache_cc: 1  # use collision checking cache. default: 1
    cache_cc_size: 200  # size of collision checking cache. default: 200
  BFMTkConfigDefault:
    type: geometric::BFMT
    num_samples: 1000  # number of states that will be sampled. default: 1000
    radius_multiplier: 1.0  # multiplier for connection radius. default: 1.0
    nearest_k: 1  # use Knearest strategy. default: 1
    balanced: 0  # use balanced algorithm. default: 0
    optimality: 1  # use optimality algorithm. default: 1
    heuristics: 1  # use heuristics. default: 1
    cache_cc: 1  # use collision checking cache. default: 1
    cache_cc_size: 200  # size of collision checking cache. default: 200
  PDSTkConfigDefault:
    type: geometric::PDST
  STRIDEkConfigDefault:
    type: geometric::STRIDE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
    use_projected_distance: 0  # whether to use projected distance. default: 0
    degree: 16  # degree of the cspace to explore. default: 16
    min_degree: 12  # min degree of the cspace to explore. default: 12
    max_degree: 18  # max degree of the cspace to explore. default: 18
    max_fail: 50  # max consecutive failures before increasing delta. default: 50
  BiTRRTkConfigDefault:
    type: geometric::BiTRRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    temp_change_factor: 0.1  # how much to increase or decrease temp. default: 2.0
    init_temperature: 100  # initial temperature. default: 100
    frountier_threshold: 0.0  # dist new state to nearest neighbor to disqualify as frontier. default: 0.0 set in setup()
    frountierNodeRatio: 0.1  # 1/10, or 1 non-frontier for every 10 frontier. default: 0.1
    cost_threshold: 1e300  # the cost threshold. motions with higher cost are discarded. default: inf
  LBTRRTkConfigDefault:
    type: geometric::LBTRRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
    epsilon: 0.4  # optimality approximation factor. default: 0.4
  BiESTkConfigDefault:
    type: geometric::BiEST
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
  ProjESTkConfigDefault:
    type: geometric::ProjEST
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
  LazyPRMkConfigDefault:
    type: geometric::LazyPRM
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
  LazyPRMstarkConfigDefault:
    type: geometric::LazyPRMstar
  SPARSkConfigDefault:
    type: geometric::SPARS
    stretch_factor: 3.0  # roadmap spanner stretch factor. default: 3.0
    sparse_delta_fraction: 0.25  # delta fraction for connection distance. default: 0.25
    dense_delta_fraction: 0.001  # delta fraction for interface detection. default: 0.001
    max_failures: 1000  # maximum consecutive failures. default: 1000
  SPARStwokConfigDefault:
    type: geometric::SPARStwo
    stretch_factor: 3.0  # roadmap spanner stretch factor. default: 3.0
    sparse_delta_fraction: 0.25  # delta fraction for connection distance. default: 0.25
    dense_delta_fraction: 0.001  # delta fraction for interface detection. default: 0.001
    max_failures: 5000  # maximum consecutive failures. default: 5000

right_arm:
  planner_configs:
    - SBLkConfigDefault
    - ESTkConfigDefault
    - LBKPIECEkConfigDefault
    - BKPIECEkConfigDefault
    - KPIECEkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - TRRTkConfigDefault
    - PRMkConfigDefault
    - PRMstarkConfigDefault
    - FMTkConfigDefault
    - BFMTkConfigDefault
    - PDSTkConfigDefault
    - STRIDEkConfigDefault
    - BiTRRTkConfigDefault
    - LBTRRTkConfigDefault
    - BiESTkConfigDefault
    - ProjESTkConfigDefault
    - LazyPRMkConfigDefault
    - LazyPRMstarkConfigDefault
    - SPARSkConfigDefault
    - SPARStwokConfigDefault
  projection_evaluator: joints(right_shoulder_pitch_joint,right_shoulder_roll_joint)
  longest_valid_segment_fraction: 0.005

left_arm:
  planner_configs:
    - SBLkConfigDefault
    - ESTkConfigDefault
    - LBKPIECEkConfigDefault
    - BKPIECEkConfigDefault
    - KPIECEkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - TRRTkConfigDefault
    - PRMkConfigDefault
    - PRMstarkConfigDefault
    - FMTkConfigDefault
    - BFMTkConfigDefault
    - PDSTkConfigDefault
    - STRIDEkConfigDefault
    - BiTRRTkConfigDefault
    - LBTRRTkConfigDefault
    - BiESTkConfigDefault
    - ProjESTkConfigDefault
    - LazyPRMkConfigDefault
    - LazyPRMstarkConfigDefault
    - SPARSkConfigDefault
    - SPARStwokConfigDefault
  projection_evaluator: joints(left_shoulder_pitch_joint,left_shoulder_roll_joint)
  longest_valid_segment_fraction: 0.005
```

## Manipulation Node Implementation

### Grasp Planning and Execution

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
from tf2_ros import Buffer, TransformListener
import json

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')

        # Initialize action client for MoveIt
        self.move_group_client = ActionClient(self, MoveGroup, 'move_group')

        # Initialize TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        self.visualization_pub = self.create_publisher(
            MarkerArray,
            '/manipulation_visualization',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/manipulation_status',
            10
        )

        # Subscribers
        self.object_pose_sub = self.create_subscription(
            String,  # In practice, this would be a more specific message type
            '/object_poses',
            self.object_pose_callback,
            10
        )

        # Service clients for gripper control
        self.gripper_clients = {
            'left': self.create_client(String, '/left_gripper/control'),
            'right': self.create_client(String, '/right_gripper/control')
        }

        # Configuration parameters
        self.declare_parameter('approach_distance', 0.15)
        self.declare_parameter('grasp_distance', 0.05)
        self.declare_parameter('lift_distance', 0.1)
        self.declare_parameter('arm_preference', 'right')  # Which arm to prefer

        self.approach_distance = self.get_parameter('approach_distance').value
        self.grasp_distance = self.get_parameter('grasp_distance').value
        self.lift_distance = self.get_parameter('lift_distance').value
        self.arm_preference = self.get_parameter('arm_preference').value

        # Current state
        self.current_objects = {}
        self.active_manipulation = False

        self.get_logger().info("Manipulation Controller initialized")

    def object_pose_callback(self, msg):
        """Receive object pose information for manipulation planning"""
        try:
            object_data = json.loads(msg.data)

            # Store object information
            for obj in object_data.get('objects', []):
                obj_id = obj.get('id', 'unknown')
                self.current_objects[obj_id] = {
                    'pose': obj.get('pose', {}),
                    'class': obj.get('class', 'unknown'),
                    'confidence': obj.get('confidence', 0.0),
                    'bbox': obj.get('bbox', {})
                }

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in object pose message")

    def plan_grasp(self, object_id, arm=None):
        """Plan a grasp for the specified object using the specified arm"""
        if arm is None:
            arm = self.arm_preference

        if object_id not in self.current_objects:
            self.get_logger().error(f"Object {object_id} not found in current objects")
            return False

        obj_info = self.current_objects[object_id]
        obj_pose = obj_info['pose']

        # Determine grasp approach and grasp poses based on object properties
        grasp_poses = self.calculate_grasp_poses(obj_pose, obj_info['class'])

        if not grasp_poses:
            self.get_logger().error(f"Could not calculate grasp poses for {object_id}")
            return False

        # Plan and execute grasp sequence
        success = self.execute_grasp_sequence(grasp_poses, arm, obj_info['class'])
        return success

    def calculate_grasp_poses(self, object_pose, object_class):
        """Calculate approach and grasp poses based on object properties"""
        # Convert object pose to position and orientation
        obj_position = np.array([
            object_pose.get('position', {}).get('x', 0.0),
            object_pose.get('position', {}).get('y', 0.0),
            object_pose.get('position', {}).get('z', 0.0)
        ])

        obj_orientation = np.array([
            object_pose.get('orientation', {}).get('x', 0.0),
            object_pose.get('orientation', {}).get('y', 0.0),
            object_pose.get('orientation', {}).get('z', 0.0),
            object_pose.get('orientation', {}).get('w', 1.0)
        ])

        # Calculate approach pose (above the object)
        approach_position = obj_position.copy()
        approach_position[2] += self.approach_distance  # Lift above object

        # Calculate grasp pose (at object level)
        grasp_position = obj_position.copy()
        grasp_position[2] += self.grasp_distance  # Slightly above surface

        # For now, use same orientation as object
        # In practice, you'd calculate appropriate grasp orientation based on object shape
        grasp_poses = [
            {
                'name': 'approach',
                'position': approach_position.tolist(),
                'orientation': obj_orientation.tolist()
            },
            {
                'name': 'grasp',
                'position': grasp_position.tolist(),
                'orientation': obj_orientation.tolist()
            },
            {
                'name': 'lift',
                'position': [grasp_position[0], grasp_position[1], grasp_position[2] + self.lift_distance],
                'orientation': obj_orientation.tolist()
            }
        ]

        return grasp_poses

    def execute_grasp_sequence(self, grasp_poses, arm, object_class):
        """Execute the complete grasp sequence: approach -> grasp -> lift"""
        self.active_manipulation = True
        success = True

        try:
            # 1. Move to approach position
            self.get_logger().info(f"Moving {arm} arm to approach position")
            approach_pose = self.create_pose_from_dict(grasp_poses[0])
            if not self.move_to_pose(approach_pose, arm):
                self.get_logger().error("Failed to move to approach position")
                success = False

            if success:
                # 2. Move to grasp position
                self.get_logger().info(f"Moving {arm} arm to grasp position")
                grasp_pose = self.create_pose_from_dict(grasp_poses[1])
                if not self.move_to_pose(grasp_pose, arm):
                    self.get_logger().error("Failed to move to grasp position")
                    success = False

            if success:
                # 3. Close gripper to grasp object
                self.get_logger().info(f"Closing {arm} gripper")
                if not self.control_gripper(arm, 'close'):
                    self.get_logger().error("Failed to close gripper")
                    success = False

            if success:
                # 4. Lift object
                self.get_logger().info(f"Lifting object with {arm} arm")
                lift_pose = self.create_pose_from_dict(grasp_poses[2])
                if not self.move_to_pose(lift_pose, arm):
                    self.get_logger().error("Failed to lift object")
                    success = False

            # Update status
            status_msg = String()
            status_msg.data = f"Grasp {'successful' if success else 'failed'} for {object_class}"
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error in grasp sequence: {e}")
            success = False
        finally:
            self.active_manipulation = False

        return success

    def move_to_pose(self, pose, arm):
        """Move specified arm to the given pose using MoveIt"""
        # Wait for the action server to be available
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return False

        # Create goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = f"{arm}_arm"

        # Set target pose
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        # Add pose constraint
        pose_constraint = PositionConstraint()
        pose_constraint.header.frame_id = "base_link"
        pose_constraint.link_name = f"{arm}_hand"
        pose_constraint.target_point_offset.x = 0.0
        pose_constraint.target_point_offset.y = 0.0
        pose_constraint.target_point_offset.z = 0.0
        pose_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pose_constraint.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pose_constraint]))

        # Send goal
        future = self.move_group_client.send_goal_async(goal_msg)

        # Wait for result (with timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

                if result_future.result() is not None:
                    result = result_future.result().result
                    return result.error_code.val == 1  # SUCCESS
                else:
                    self.get_logger().error("Result future timed out")
                    return False
            else:
                self.get_logger().error("Goal was rejected")
                return False
        else:
            self.get_logger().error("Failed to send goal")
            return False

    def control_gripper(self, arm, action):
        """Control the specified gripper (open/close)"""
        if arm not in self.gripper_clients:
            self.get_logger().error(f"No gripper client for {arm}")
            return False

        client = self.gripper_clients[arm]
        if not client.service_is_ready():
            self.get_logger().error(f"Gripper service for {arm} not ready")
            return False

        request = String()
        request.data = action  # 'open' or 'close'

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return True
        else:
            self.get_logger().error(f"Failed to {action} {arm} gripper")
            return False

    def place_object(self, target_pose, arm=None):
        """Place the currently held object at the target pose"""
        if arm is None:
            arm = self.arm_preference

        # Calculate placement sequence: move to target -> open gripper -> lift
        placement_poses = [
            {
                'name': 'approach_place',
                'position': [
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z + self.approach_distance
                ],
                'orientation': [
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w
                ]
            },
            {
                'name': 'place',
                'position': [
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z + self.grasp_distance
                ],
                'orientation': [
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w
                ]
            },
            {
                'name': 'retreat',
                'position': [
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z + self.approach_distance
                ],
                'orientation': [
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w
                ]
            }
        ]

        success = True
        self.active_manipulation = True

        try:
            # 1. Move to place approach position
            approach_pose = self.create_pose_from_dict(placement_poses[0])
            if not self.move_to_pose(approach_pose, arm):
                self.get_logger().error("Failed to move to place approach position")
                success = False

            if success:
                # 2. Move to place position
                place_pose = self.create_pose_from_dict(placement_poses[1])
                if not self.move_to_pose(place_pose, arm):
                    self.get_logger().error("Failed to move to place position")
                    success = False

            if success:
                # 3. Open gripper to place object
                self.get_logger().info(f"Opening {arm} gripper to place object")
                if not self.control_gripper(arm, 'open'):
                    self.get_logger().error("Failed to open gripper")
                    success = False

            if success:
                # 4. Retreat from object
                retreat_pose = self.create_pose_from_dict(placement_poses[2])
                if not self.move_to_pose(retreat_pose, arm):
                    self.get_logger().error("Failed to retreat from placed object")
                    success = False

            # Update status
            status_msg = String()
            status_msg.data = f"Placement {'successful' if success else 'failed'}"
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error in placement sequence: {e}")
            success = False
        finally:
            self.active_manipulation = False

        return success

    def create_pose_from_dict(self, pose_dict):
        """Create a Pose message from a dictionary"""
        pose = Pose()
        pose.position.x = pose_dict['position'][0]
        pose.position.y = pose_dict['position'][1]
        pose.position.z = pose_dict['position'][2]

        if len(pose_dict['orientation']) >= 4:
            pose.orientation.x = pose_dict['orientation'][0]
            pose.orientation.y = pose_dict['orientation'][1]
            pose.orientation.z = pose_dict['orientation'][2]
            pose.orientation.w = pose_dict['orientation'][3]
        else:
            # Default orientation (identity quaternion)
            pose.orientation.w = 1.0

        return pose

    def add_collision_object(self, object_id, pose, shape_type, dimensions):
        """Add a collision object to the planning scene"""
        co = CollisionObject()
        co.header.frame_id = "map"
        co.id = object_id

        # Define the primitive shape
        primitive = SolidPrimitive()
        primitive.type = shape_type
        primitive.dimensions = dimensions

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        self.collision_object_pub.publish(co)

    def visualize_manipulation_plan(self, grasp_poses):
        """Visualize the manipulation plan in RViz"""
        marker_array = MarkerArray()

        for i, pose_dict in enumerate(grasp_poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grasp_poses"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = pose_dict['position'][0]
            marker.pose.position.y = pose_dict['position'][1]
            marker.pose.position.z = pose_dict['position'][2]

            # Set orientation
            marker.pose.orientation.x = pose_dict['orientation'][0]
            marker.pose.orientation.y = pose_dict['orientation'][1]
            marker.pose.orientation.z = pose_dict['orientation'][2]
            marker.pose.orientation.w = pose_dict['orientation'][3]

            # Set scale
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Set color (different for each pose)
            if pose_dict['name'] == 'approach':
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif pose_dict['name'] == 'grasp':
                marker.color.r = 0.0  # Green
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # lift
                marker.color.r = 0.0  # Blue
                marker.color.g = 0.0
                marker.color.b = 1.0

            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.visualization_pub.publish(marker_array)
```

## Integration with Voice Commands

### Voice-Driven Manipulation

```python
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class VoiceManipulationController(Node):
    def __init__(self):
        super().__init__('voice_manipulation_controller')

        # Initialize manipulation controller
        self.manipulation_controller = ManipulationController()

        # Subscribe to processed commands
        self.command_sub = self.create_subscription(
            String,
            '/processed_command',
            self.processed_command_callback,
            10
        )

        # Publishers for status updates
        self.status_pub = self.create_publisher(
            String,
            '/manipulation_status',
            10
        )

        self.get_logger().info("Voice Manipulation Controller initialized")

    def processed_command_callback(self, msg):
        """Process voice commands that involve manipulation"""
        try:
            command_data = json.loads(msg.data)

            if command_data.get('action_type') == 'manipulation':
                self.execute_manipulation_command(command_data)
            elif command_data.get('action_type') == 'navigation_and_manipulation':
                self.execute_navigation_and_manipulation_command(command_data)

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in command message")

    def execute_manipulation_command(self, command_data):
        """Execute a manipulation command from voice processing"""
        target_object = command_data.get('target_object')
        action_type = command_data.get('manipulation_action', 'grasp')
        target_location = command_data.get('target_location')

        if not target_object:
            self.get_logger().error("No target object specified in manipulation command")
            return

        # Find the object in current perception
        object_id = self.find_object_by_name(target_object)

        if not object_id:
            self.get_logger().error(f"Could not find object '{target_object}' in current perception")
            # Could trigger object search here
            return

        if action_type == 'grasp':
            success = self.manipulation_controller.plan_grasp(object_id)
            if success:
                self.get_logger().info(f"Successfully grasped {target_object}")
                self.announce_completion(f"Successfully picked up the {target_object}")
            else:
                self.get_logger().error(f"Failed to grasp {target_object}")
                self.announce_completion(f"Sorry, I couldn't pick up the {target_object}")

        elif action_type == 'place':
            if target_location:
                # Need to navigate to target location first, then place
                self.get_logger().info(f"Need to navigate to {target_location} to place {target_object}")
                # This would involve coordinating with navigation system
            else:
                self.get_logger().error("No target location specified for place action")

        elif action_type == 'move':
            if target_location:
                # Pick up object, navigate to location, then place
                self.get_logger().info(f"Moving {target_object} to {target_location}")
                # This would be a complex sequence involving grasp, navigation, and place
            else:
                self.get_logger().error("No target location specified for move action")

    def find_object_by_name(self, object_name):
        """Find an object ID by its name from current perception"""
        # Search in current objects maintained by manipulation controller
        for obj_id, obj_info in self.manipulation_controller.current_objects.items():
            if obj_info['class'].lower() == object_name.lower():
                return obj_id

        # If not found by exact class, try fuzzy matching
        for obj_id, obj_info in self.manipulation_controller.current_objects.items():
            if object_name.lower() in obj_info['class'].lower():
                return obj_id

        return None

    def announce_completion(self, message):
        """Announce completion of manipulation task"""
        # This would publish to text-to-speech system
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
```

## Manipulation Planning Algorithms

### Grasp Pose Generator

```python
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R

class GraspPoseGenerator:
    def __init__(self):
        # Define standard grasp types
        self.grasp_types = {
            'top_grasp': {
                'approach_direction': [0, 0, -1],  # From above
                'grasp_direction': [0, 0, 1],     # Grasping upward
                'gripper_orientation': [0, 0, 0, 1]  # Identity quaternion
            },
            'side_grasp': {
                'approach_direction': [1, 0, 0],  # From side (positive X)
                'grasp_direction': [0, 0, 0],     # Perpendicular to approach
                'gripper_orientation': [0, 0, 0, 1]
            },
            'pinch_grasp': {
                'approach_direction': [0, 1, 0],  # From side (positive Y)
                'grasp_direction': [0, 0, 0],
                'gripper_orientation': [0, 0, 0, 1]
            }
        }

    def generate_grasps_for_object(self, object_pose, object_class, object_dimensions):
        """Generate potential grasp poses for an object based on its properties"""
        grasps = []

        # Determine appropriate grasp types based on object class
        object_grasp_types = self.get_appropriate_grasps(object_class, object_dimensions)

        for grasp_type in object_grasp_types:
            grasp_poses = self.generate_grasps_for_type(
                object_pose,
                object_class,
                object_dimensions,
                grasp_type
            )
            grasps.extend(grasp_poses)

        return grasps

    def get_appropriate_grasps(self, object_class, object_dimensions):
        """Determine appropriate grasp types for an object class"""
        # Map object classes to appropriate grasp types
        class_to_grasps = {
            'bottle': ['top_grasp', 'side_grasp'],
            'cup': ['side_grasp', 'pinch_grasp'],
            'book': ['side_grasp', 'top_grasp'],
            'box': ['top_grasp', 'side_grasp'],
            'cell_phone': ['pinch_grasp', 'side_grasp'],
            'can': ['side_grasp', 'top_grasp'],
            'bowl': ['side_grasp'],
            'plate': ['top_grasp']
        }

        # Default to side grasp for unknown objects
        return class_to_grasps.get(object_class, ['side_grasp'])

    def generate_grasps_for_type(self, object_pose, object_class, object_dimensions, grasp_type):
        """Generate grasp poses for a specific grasp type"""
        grasps = []

        # Get grasp configuration
        config = self.grasp_types[grasp_type]

        # Calculate grasp positions based on object dimensions
        object_center = np.array([
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z
        ])

        # Calculate approach and grasp positions
        approach_offset = np.array(config['approach_direction']) * 0.15  # 15cm approach distance
        grasp_offset = np.array(config['approach_direction']) * 0.05   # 5cm grasp distance

        approach_position = object_center + approach_offset
        grasp_position = object_center + grasp_offset

        # Create approach pose
        approach_pose = Pose()
        approach_pose.position.x = approach_position[0]
        approach_pose.position.y = approach_position[1]
        approach_pose.position.z = approach_position[2]

        # Set orientation based on grasp type
        approach_pose.orientation.x = config['gripper_orientation'][0]
        approach_pose.orientation.y = config['gripper_orientation'][1]
        approach_pose.orientation.z = config['gripper_orientation'][2]
        approach_pose.orientation.w = config['gripper_orientation'][3]

        # Create grasp pose (same position, potentially different orientation)
        grasp_pose = Pose()
        grasp_pose.position.x = grasp_position[0]
        grasp_pose.position.y = grasp_position[1]
        grasp_pose.position.z = grasp_position[2]

        # Same orientation as approach for now (in practice, this might vary)
        grasp_pose.orientation.x = approach_pose.orientation.x
        grasp_pose.orientation.y = approach_pose.orientation.y
        grasp_pose.orientation.z = approach_pose.orientation.z
        grasp_pose.orientation.w = approach_pose.orientation.w

        # Create lift pose (above grasp position)
        lift_position = grasp_position + np.array([0, 0, 0.1])  # Lift 10cm
        lift_pose = Pose()
        lift_pose.position.x = lift_position[0]
        lift_pose.position.y = lift_position[1]
        lift_pose.position.z = lift_position[2]
        lift_pose.orientation.x = approach_pose.orientation.x
        lift_pose.orientation.y = approach_pose.orientation.y
        lift_pose.orientation.z = approach_pose.orientation.z
        lift_pose.orientation.w = approach_pose.orientation.w

        # Add to grasps list
        grasps.append({
            'type': grasp_type,
            'approach_pose': approach_pose,
            'grasp_pose': grasp_pose,
            'lift_pose': lift_pose,
            'confidence': 0.8  # Base confidence
        })

        return grasps

    def rank_grasps(self, grasps, robot_state):
        """Rank grasps based on accessibility and robot configuration"""
        ranked_grasps = []

        for grasp in grasps:
            # Calculate score based on various factors
            score = self.calculate_grasp_score(grasp, robot_state)
            ranked_grasps.append((grasp, score))

        # Sort by score (descending)
        ranked_grasps.sort(key=lambda x: x[1], reverse=True)

        return [grasp for grasp, score in ranked_grasps]

    def calculate_grasp_score(self, grasp, robot_state):
        """Calculate a score for a grasp based on various factors"""
        score = 0.0

        # Factor 1: Accessibility (distance from robot)
        robot_pos = np.array([robot_state['position']['x'],
                             robot_state['position']['y'],
                             robot_state['position']['z']])
        grasp_pos = np.array([grasp['grasp_pose'].position.x,
                              grasp['grasp_pose'].position.y,
                              grasp['grasp_pose'].position.z])

        distance = np.linalg.norm(robot_pos - grasp_pos)
        # Score decreases with distance (max 1m reach)
        distance_score = max(0, (1.0 - distance/1.0))
        score += distance_score * 0.3

        # Factor 2: Grasp stability (based on grasp type and object)
        stability_score = grasp['confidence']
        score += stability_score * 0.4

        # Factor 3: Robot configuration (avoid joint limits)
        # This would involve more complex IK calculations
        configuration_score = 0.3  # Simplified for now
        score += configuration_score * 0.3

        return score
```

## Testing and Validation

### Manipulation Test Suite

```python
import unittest
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from manipulation_controller import ManipulationController, GraspPoseGenerator

class TestManipulation(unittest.TestCase):
    def setUp(self):
        self.grasp_generator = GraspPoseGenerator()
        # Mock manipulation controller for testing
        self.controller = ManipulationController()

    def test_grasp_generation(self):
        """Test grasp pose generation for different object types"""
        # Create a test object pose
        test_pose = Pose()
        test_pose.position.x = 1.0
        test_pose.position.y = 1.0
        test_pose.position.z = 0.5
        test_pose.orientation.w = 1.0

        # Test with different object classes
        test_cases = [
            ('bottle', [0.1, 0.1, 0.2]),  # width, depth, height
            ('cup', [0.08, 0.08, 0.1]),
            ('book', [0.2, 0.15, 0.03]),
            ('cell_phone', [0.07, 0.14, 0.01])
        ]

        for obj_class, dimensions in test_cases:
            with self.subTest(obj_class=obj_class):
                grasps = self.grasp_generator.generate_grasps_for_object(
                    test_pose, obj_class, dimensions
                )

                # Should generate at least one grasp
                self.assertGreater(len(grasps), 0)

                # Check that all grasps have required fields
                for grasp in grasps:
                    self.assertIn('type', grasp)
                    self.assertIsInstance(grasp['approach_pose'], Pose)
                    self.assertIsInstance(grasp['grasp_pose'], Pose)
                    self.assertIsInstance(grasp['lift_pose'], Pose)

    def test_grasp_ranking(self):
        """Test grasp ranking functionality"""
        # Create test grasps
        test_pose = Pose()
        test_pose.position.x = 1.0
        test_pose.position.y = 1.0
        test_pose.position.z = 0.5
        test_pose.orientation.w = 1.0

        grasps = self.grasp_generator.generate_grasps_for_object(
            test_pose, 'bottle', [0.1, 0.1, 0.2]
        )

        # Mock robot state
        robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'configuration': [0.0] * 6  # Mock joint configuration
        }

        ranked_grasps = self.grasp_generator.rank_grasps(grasps, robot_state)

        # Should return same number of grasps
        self.assertEqual(len(ranked_grasps), len(grasps))

        # Should be sorted by score (descending)
        scores = [self.grasp_generator.calculate_grasp_score(g, robot_state) for g in ranked_grasps]
        self.assertEqual(scores, sorted(scores, reverse=True))

    def test_appropriate_grasps_selection(self):
        """Test that appropriate grasps are selected for different objects"""
        test_cases = [
            ('bottle', ['top_grasp', 'side_grasp']),
            ('cup', ['side_grasp', 'pinch_grasp']),
            ('book', ['side_grasp', 'top_grasp']),
            ('cell_phone', ['pinch_grasp', 'side_grasp']),
            ('unknown_object', ['side_grasp'])  # Default case
        ]

        for obj_class, expected_grasps in test_cases:
            with self.subTest(obj_class=obj_class):
                selected_grasps = self.grasp_generator.get_appropriate_grasps(
                    obj_class, [0.1, 0.1, 0.1]
                )

                # Check that expected grasps are included
                for expected in expected_grasps:
                    self.assertIn(expected, selected_grasps)

if __name__ == '__main__':
    unittest.main()
```

## Best Practices

1. **Safety First**: Always check for collisions and maintain safe distances
2. **Grasp Planning**: Consider object properties when selecting grasp types
3. **Redundancy**: Plan multiple grasp options for robustness
4. **Error Handling**: Implement recovery behaviors for failed grasps
5. **Force Control**: Use appropriate force limits to avoid damaging objects
6. **Verification**: Confirm grasp success before lifting or moving

## Integration with Capstone System

The manipulation system integrates with the overall capstone project by:

1. **Perception Integration**: Using object detection and pose estimation from vision system
2. **Navigation Coordination**: Working with navigation system for pick-and-place tasks
3. **Voice Command Processing**: Responding to manipulation commands from voice system
4. **Task Planning**: Coordinating with overall task planning system
5. **Feedback Loop**: Providing status updates to the central system

This manipulation planning system enables the humanoid robot to interact with objects in its environment as part of the capstone project, supporting the overall goal of autonomous task execution based on voice commands.