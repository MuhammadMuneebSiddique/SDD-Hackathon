---
sidebar_position: 4
title: 'Object Identification'
---

# Object Identification

This section covers the implementation of object identification for the capstone project, enabling the humanoid robot to detect, recognize, and locate objects in its environment.

## Learning Objectives

After completing this section, you will be able to:
- Implement object detection and recognition using Isaac ROS packages
- Configure perception systems for robot manipulation tasks
- Integrate vision with navigation and manipulation planning
- Handle various object types and environmental conditions
- Optimize perception systems for real-time performance

## Perception Architecture

The object identification system uses the Isaac ROS perception stack with several key components:

1. **Camera System**: RGB-D camera for capturing visual data
2. **Object Detection**: Deep learning-based object detection
3. **Pose Estimation**: 6D pose estimation for manipulation planning
4. **Scene Understanding**: Spatial relationships and scene context
5. **Tracking**: Object tracking across frames for stability

## Isaac ROS Perception Setup

### Required Isaac ROS Packages

```bash
# Install Isaac ROS perception packages
sudo apt install ros-humble-isaac-ros-dnn-inference
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-segment-any-thing
sudo apt install ros-humble-isaac-ros-point-cloud
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-image-pipeline
```

### Camera Configuration

```yaml
# config/camera_config.yaml
camera:
  ros__parameters:
    # Camera parameters
    width: 640
    height: 480
    fps: 30
    camera_info_url: "file://$(findpkg share camera_calibration)/ost.yaml"

    # Image processing
    rectify: true
    brightness: 0.0
    contrast: 1.0
    saturation: 1.0
    gamma: 1.0
    gain: 1.0
    exposure: -1  # Auto exposure

    # Output settings
    output_encoding: "rgb8"
    flip_horizontal: false
    flip_vertical: false
```

### Object Detection Configuration

```yaml
# config/object_detection.yaml
object_detection:
  ros__parameters:
    # Model parameters
    engine_file_path: "/path/to/yolov8n.plan"
    input_tensor_names: ["input_tensor"]
    input_tensor_formats: ["nitros_tensor_list_nchw_rgb_f32"]
    output_tensor_names: ["detections"]
    output_tensor_formats: ["nitros_tensor_list_nhwc_rgb_f32"]

    # Network parameters
    network_image_width: 640
    network_image_height: 640
    network_image_type: "nitros_image_rgb8"
    network_normalization_scale: 0.0039215697984695435
    network_mean_pixel_value_red: 0.0
    network_mean_pixel_value_green: 0.0
    network_mean_pixel_value_blue: 0.0

    # Detection parameters
    confidence_threshold: 0.5
    maximum_allowed_latency: 100
    num_frames: 100
    input_qos_depth: 5
    input_qos_reliability_policy: "reliable"
    input_qos_history_policy: "keep_last"
    input_qos_deadline: {sec: 1, nsec: 0}
    output_qos_depth: 5
    output_qos_reliability_policy: "reliable"
    output_qos_history_policy: "keep_last"
    output_qos_deadline: {sec: 1, nsec: 0}

    # Input/Output mapping
    attach_to_shared_component: false
    component_name: ""
    input_topic_name: "image"
    output_topic_name: "detections"
```

## Object Detection Node Implementation

### Object Detection Class

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Dict, Optional
import json

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/object_detection/debug_image',
            10
        )

        # Object classes (COCO dataset classes)
        self.coco_classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Object colors for visualization
        self.colors = np.random.randint(0, 255, size=(len(self.coco_classes), 3))

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False

        # Configuration parameters
        self.confidence_threshold = 0.5
        self.debug_visualization = True

        self.get_logger().info("Object Detection Node initialized")

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera calibration parameters received")

    def image_callback(self, msg):
        """Process incoming camera images for object detection"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Perform object detection
            detections = self.perform_detection(cv_image)

            # Create Detection2DArray message
            detection_array = self.create_detection_message(detections, msg.header)

            # Publish detections
            self.detection_pub.publish(detection_array)

            # Publish debug visualization if enabled
            if self.debug_visualization:
                debug_img = self.draw_detections(cv_image, detections)
                debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_img, encoding='rgb8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def perform_detection(self, image):
        """Perform object detection on the input image"""
        # In practice, this would use Isaac ROS DNN inference
        # For demonstration, we'll use a mock detection function
        return self.mock_detection(image)

    def mock_detection(self, image):
        """Mock detection function for demonstration"""
        h, w = image.shape[:2]

        # Create mock detections
        detections = []

        # Example: detect some common objects
        if np.random.random() > 0.3:  # Add person detection sometimes
            detections.append({
                'class_id': 0,  # person
                'class_name': 'person',
                'confidence': np.random.uniform(0.7, 0.95),
                'bbox': [int(w*0.3), int(h*0.2), int(w*0.2), int(h*0.6)],  # [x, y, width, height]
                'center_3d': None  # Will be populated with depth info
            })

        if np.random.random() > 0.4:  # Add cup detection sometimes
            detections.append({
                'class_id': 41,  # cup
                'class_name': 'cup',
                'confidence': np.random.uniform(0.6, 0.9),
                'bbox': [int(w*0.6), int(h*0.4), int(w*0.1), int(h*0.1)],
                'center_3d': None
            })

        if np.random.random() > 0.5:  # Add bottle detection sometimes
            detections.append({
                'class_id': 39,  # bottle
                'class_name': 'bottle',
                'confidence': np.random.uniform(0.55, 0.85),
                'bbox': [int(w*0.1), int(h*0.3), int(w*0.1), int(h*0.2)],
                'center_3d': None
            })

        return detections

    def create_detection_message(self, detections, header):
        """Create Detection2DArray message from detections"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            if det['confidence'] >= self.confidence_threshold:
                detection = Detection2D()

                # Set bounding box
                bbox = det['bbox']
                detection.bbox.center.x = bbox[0] + bbox[2] / 2  # center x
                detection.bbox.center.y = bbox[1] + bbox[3] / 2  # center y
                detection.bbox.size_x = bbox[2]  # width
                detection.bbox.size_y = bbox[3]  # height

                # Set hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(det['class_id'])
                hypothesis.hypothesis.score = det['confidence']

                detection.results.append(hypothesis)

                detection_array.detections.append(detection)

        return detection_array

    def draw_detections(self, image, detections):
        """Draw detections on image for visualization"""
        vis_image = image.copy()

        for det in detections:
            if det['confidence'] >= self.confidence_threshold:
                bbox = det['bbox']
                class_id = det['class_id']

                # Get color for this class
                color = tuple(map(int, self.colors[class_id]))

                # Draw bounding box
                pt1 = (bbox[0], bbox[1])
                pt2 = (bbox[0] + bbox[2], bbox[1] + bbox[3])
                cv2.rectangle(vis_image, pt1, pt2, color, 2)

                # Draw label
                label = f"{det['class_name']}: {det['confidence']:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]

                label_pt1 = (bbox[0], bbox[1] - label_size[1] - 10)
                label_pt2 = (bbox[0] + label_size[0], bbox[1])

                cv2.rectangle(vis_image, label_pt1, label_pt2, color, -1)
                cv2.putText(
                    vis_image, label, (bbox[0], bbox[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
                )

        return vis_image
```

## 3D Object Pose Estimation

### Pose Estimation Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            Detection2DArray,  # Extended with 3D poses
            '/object_poses',
            10
        )

        # Camera parameters (will be set from camera info)
        self.camera_matrix = np.eye(3)
        self.depth_image = None
        self.pointcloud = None

        # Configuration
        self.use_pointcloud_for_pose = True  # Use point cloud if available, otherwise use depth

        self.get_logger().info("Pose Estimation Node initialized")

    def detection_callback(self, msg):
        """Process detections and estimate 3D poses"""
        try:
            # Process each detection to add 3D pose information
            enhanced_detections = Detection2DArray()
            enhanced_detections.header = msg.header

            for detection in msg.detections:
                enhanced_detection = self.estimate_pose_3d(detection)
                enhanced_detections.detections.append(enhanced_detection)

            # Publish enhanced detections with 3D poses
            self.pose_pub.publish(enhanced_detections)

        except Exception as e:
            self.get_logger().error(f"Error in detection callback: {e}")

    def depth_callback(self, msg):
        """Receive depth image"""
        try:
            # Convert ROS Image to numpy array
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def pointcloud_callback(self, msg):
        """Receive point cloud data"""
        try:
            # Store point cloud for later use
            self.pointcloud = msg
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def estimate_pose_3d(self, detection):
        """Estimate 3D pose from 2D detection and depth information"""
        # Create a copy of the detection
        enhanced_detection = Detection2D()
        enhanced_detection.header = detection.header
        enhanced_detection.bbox = detection.bbox
        enhanced_detection.results = detection.results

        # Get the center pixel of the bounding box
        center_x = int(detection.bbox.center.x)
        center_y = int(detection.bbox.center.y)

        # Get 3D position from depth
        if self.use_pointcloud_for_pose and self.pointcloud:
            # Use point cloud data for more accurate 3D position
            pos_3d = self.get_position_from_pointcloud(center_x, center_y)
        else:
            # Use depth image
            pos_3d = self.get_position_from_depth(center_x, center_y)

        if pos_3d is not None:
            # Add 3D pose information to detection results
            # For now, we'll just store the 3D position in the detection
            # In practice, you'd want to create a more sophisticated pose representation

            # The position can be stored as additional metadata
            # For demonstration, we'll just log it
            self.get_logger().debug(f"Estimated 3D position: {pos_3d}")

        return enhanced_detection

    def get_position_from_depth(self, u, v):
        """Get 3D position from depth image and pixel coordinates"""
        if self.depth_image is None:
            return None

        if u < 0 or u >= self.depth_image.shape[1] or v < 0 or v >= self.depth_image.shape[0]:
            return None

        # Get depth value
        depth = self.depth_image[v, u]

        if np.isnan(depth) or np.isinf(depth):
            return None

        # Convert pixel coordinates to 3D using camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Apply inverse camera projection
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return np.array([x, y, z])

    def get_position_from_pointcloud(self, u, v):
        """Get 3D position from point cloud at pixel coordinates"""
        if self.pointcloud is None:
            return self.get_position_from_depth(u, v)  # Fallback to depth

        try:
            # Convert PointCloud2 to list of points
            points = list(point_cloud2.read_points(
                self.pointcloud,
                field_names=("x", "y", "z"),
                skip_nans=True
            ))

            # Find the point closest to pixel (u, v)
            # This is a simplified approach - in practice you'd want to
            # implement a more sophisticated mapping between image pixels and point cloud points
            # based on the camera's projection model

            # For now, we'll use a simple approach assuming dense point cloud
            # where each pixel has a corresponding point
            width = self.pointcloud.width
            height = self.pointcloud.height

            if u < width and v < height:
                idx = v * width + u
                if idx < len(points):
                    point = points[idx]
                    return np.array([point[0], point[1], point[2]])

            return None
        except Exception as e:
            self.get_logger().error(f"Error getting position from point cloud: {e}")
            return self.get_position_from_depth(u, v)  # Fallback
```

## Object Recognition with Isaac ROS

### Isaac ROS Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from isaac_ros_tensor_list_interfaces.msg import TensorList
from isaac_ros_dnn_inference_interfaces.msg import Detections
from cv_bridge import CvBridge
import numpy as np

class IsaacObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__('isaac_object_recognition_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Isaac ROS specific publishers/subscribers
        self.tensor_pub = self.create_publisher(
            TensorList,
            '/tensor_pub',
            10
        )

        self.tensor_sub = self.create_publisher(
            TensorList,
            '/tensor_sub',
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        # Parameters
        self.declare_parameter('model_path', '/path/to/model.plan')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_objects', 10)

        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.max_objects = self.get_parameter('max_objects').value

        self.get_logger().info("Isaac Object Recognition Node initialized")

    def detection_callback(self, msg):
        """Process detections from Isaac ROS DNN inference"""
        filtered_detections = self.filter_detections_by_confidence(msg)
        self.process_filtered_detections(filtered_detections)

    def filter_detections_by_confidence(self, detection_array):
        """Filter detections based on confidence threshold"""
        filtered_array = Detection2DArray()
        filtered_array.header = detection_array.header

        for detection in detection_array.detections:
            # Check if any result meets confidence threshold
            high_confidence_results = [
                result for result in detection.results
                if result.hypothesis.score >= self.confidence_threshold
            ]

            if high_confidence_results:
                # Create new detection with only high-confidence results
                new_detection = Detection2D()
                new_detection.header = detection.header
                new_detection.bbox = detection.bbox
                new_detection.results = high_confidence_results
                filtered_array.detections.append(new_detection)

        return filtered_array

    def process_filtered_detections(self, detection_array):
        """Process filtered detections for the capstone project"""
        # In the capstone context, we might want to:
        # 1. Identify objects relevant to the current task
        # 2. Filter out irrelevant objects
        # 3. Prepare object information for manipulation planning

        relevant_objects = []
        task_relevant_classes = ['bottle', 'cup', 'book', 'cell phone', 'laptop', 'box']

        for detection in detection_array.detections:
            # Get the most likely class for this detection
            if detection.results:
                best_result = max(detection.results, key=lambda x: x.hypothesis.score)
                class_name = best_result.hypothesis.class_id

                # Check if this class is relevant to our task
                if class_name in task_relevant_classes:
                    object_info = {
                        'class': class_name,
                        'confidence': best_result.hypothesis.score,
                        'bbox_center': (detection.bbox.center.x, detection.bbox.center.y),
                        'bbox_size': (detection.bbox.size_x, detection.bbox.size_y)
                    }
                    relevant_objects.append(object_info)

        # Log relevant objects found
        if relevant_objects:
            self.get_logger().info(f"Found {len(relevant_objects)} task-relevant objects:")
            for obj in relevant_objects:
                self.get_logger().info(f"  - {obj['class']} (conf: {obj['confidence']:.2f})")

        # In a real implementation, you would publish this information
        # to a task planning node
        self.publish_relevant_objects(relevant_objects)

    def publish_relevant_objects(self, objects):
        """Publish relevant objects for task planning"""
        # This would publish to a task planning node
        # For now, we'll just log the objects
        pass
```

## Integration with Manipulation Planning

### Object-Action Mapper

```python
class ObjectActionMapper:
    def __init__(self):
        # Define object-action relationships
        self.object_manipulation_map = {
            'bottle': {
                'grasp_type': 'power_grasp',
                'approach_direction': 'top_down',
                'grasp_points': ['center_top', 'sides'],
                'manipulation_actions': ['pick_up', 'pour', 'place']
            },
            'cup': {
                'grasp_type': 'pinch_grasp',
                'approach_direction': 'side_approach',
                'grasp_points': ['handle', 'rim'],
                'manipulation_actions': ['pick_up', 'carry', 'place']
            },
            'book': {
                'grasp_type': 'power_grasp',
                'approach_direction': 'edge_approach',
                'grasp_points': ['spine', 'edges'],
                'manipulation_actions': ['pick_up', 'stack', 'place']
            },
            'cell phone': {
                'grasp_type': 'pinch_grasp',
                'approach_direction': 'top_down',
                'grasp_points': ['sides', 'bottom_edge'],
                'manipulation_actions': ['pick_up', 'inspect', 'place']
            }
        }

    def get_manipulation_plan(self, object_class, object_pose):
        """Get manipulation plan for a detected object"""
        if object_class not in self.object_manipulation_map:
            return None

        obj_info = self.object_manipulation_map[object_class]

        # Create a manipulation plan based on object properties
        manipulation_plan = {
            'object_class': object_class,
            'grasp_type': obj_info['grasp_type'],
            'approach_direction': obj_info['approach_direction'],
            'grasp_points': obj_info['grasp_points'],
            'actions': obj_info['manipulation_actions'],
            'estimated_difficulty': self.estimate_manipulation_difficulty(object_class, object_pose)
        }

        return manipulation_plan

    def estimate_manipulation_difficulty(self, object_class, object_pose):
        """Estimate difficulty of manipulating the object"""
        # Factors affecting difficulty:
        # - Object size and weight (approximated by class)
        # - Object pose (orientation, stability)
        # - Accessibility (based on location in scene)
        # - Fragility (by object class)

        base_difficulty = {
            'bottle': 0.6,  # Medium difficulty - needs careful gripping
            'cup': 0.5,     # Medium difficulty - handle requires precision
            'book': 0.3,    # Low difficulty - stable shape
            'cell phone': 0.8,  # High difficulty - fragile, precise gripping needed
            'box': 0.4      # Low to medium - depends on size
        }.get(object_class, 0.7)  # Default to medium-high difficulty

        # Adjust based on pose factors
        # For now, we'll return the base difficulty
        # In practice, you'd analyze the pose to adjust difficulty

        return base_difficulty
```

## Scene Understanding

### Scene Analysis Node

```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
import json
from typing import List, Dict

class SceneAnalysisNode(Node):
    def __init__(self):
        super().__init__('scene_analysis_node')

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_poses',  # Assuming this topic has 3D pose info
            self.scene_callback,
            10
        )

        # Publishers
        self.scene_description_pub = self.create_publisher(
            String,
            '/scene_description',
            10
        )

        self.spatial_relationships_pub = self.create_publisher(
            String,
            '/spatial_relationships',
            10
        )

        # Object-action mapper
        self.object_mapper = ObjectActionMapper()

        self.get_logger().info("Scene Analysis Node initialized")

    def scene_callback(self, msg):
        """Analyze the current scene and extract meaningful information"""
        try:
            # Convert detections to scene representation
            scene_objects = self.extract_scene_objects(msg)

            # Analyze spatial relationships
            spatial_relationships = self.analyze_spatial_relationships(scene_objects)

            # Identify potential manipulation targets
            manipulation_targets = self.identify_manipulation_targets(scene_objects)

            # Create scene description
            scene_description = self.create_scene_description(
                scene_objects,
                spatial_relationships,
                manipulation_targets
            )

            # Publish scene information
            desc_msg = String()
            desc_msg.data = json.dumps(scene_description)
            self.scene_description_pub.publish(desc_msg)

            # Publish spatial relationships separately
            rel_msg = String()
            rel_msg.data = json.dumps(spatial_relationships)
            self.spatial_relationships_pub.publish(rel_msg)

        except Exception as e:
            self.get_logger().error(f"Error in scene analysis: {e}")

    def extract_scene_objects(self, detection_array):
        """Extract meaningful object information from detections"""
        objects = []

        for detection in detection_array.detections:
            if detection.results:
                # Get the most confident result
                best_result = max(detection.results, key=lambda x: x.hypothesis.score)

                obj = {
                    'class': best_result.hypothesis.class_id,
                    'confidence': best_result.hypothesis.score,
                    'bbox_center': {
                        'x': detection.bbox.center.x,
                        'y': detection.bbox.center.y
                    },
                    'bbox_size': {
                        'width': detection.bbox.size_x,
                        'height': detection.bbox.size_y
                    },
                    'manipulation_plan': None
                }

                # Get manipulation plan if available
                manipulation_plan = self.object_mapper.get_manipulation_plan(
                    obj['class'],
                    obj['bbox_center']
                )
                obj['manipulation_plan'] = manipulation_plan

                objects.append(obj)

        return objects

    def analyze_spatial_relationships(self, objects):
        """Analyze spatial relationships between objects"""
        relationships = []

        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    # Calculate spatial relationship
                    dx = obj2['bbox_center']['x'] - obj1['bbox_center']['x']
                    dy = obj2['bbox_center']['y'] - obj1['bbox_center']['y']

                    distance = (dx**2 + dy**2)**0.5

                    # Determine spatial relationship based on distance and positions
                    if distance < 50:  # pixels, adjust based on your scale
                        relationship = {
                            'object1': obj1['class'],
                            'object2': obj2['class'],
                            'relationship': 'adjacent',
                            'distance_px': distance,
                            'vector': {'dx': dx, 'dy': dy}
                        }
                    elif distance < 150:
                        relationship = {
                            'object1': obj1['class'],
                            'object2': obj2['class'],
                            'relationship': 'nearby',
                            'distance_px': distance,
                            'vector': {'dx': dx, 'dy': dy}
                        }
                    else:
                        relationship = {
                            'object1': obj1['class'],
                            'object2': obj2['class'],
                            'relationship': 'distant',
                            'distance_px': distance,
                            'vector': {'dx': dx, 'dy': dy}
                        }

                    relationships.append(relationship)

        return relationships

    def identify_manipulation_targets(self, objects):
        """Identify objects that are suitable for manipulation"""
        targets = []

        for obj in objects:
            # Consider manipulable objects based on class and confidence
            if (obj['manipulation_plan'] is not None and
                obj['confidence'] > 0.6):  # Reasonable confidence threshold
                targets.append(obj)

        return targets

    def create_scene_description(self, objects, relationships, targets):
        """Create a comprehensive scene description"""
        scene_desc = {
            'timestamp': self.get_clock().now().nanoseconds,
            'object_count': len(objects),
            'objects': objects,
            'spatial_relationships': relationships,
            'manipulation_targets': targets,
            'summary': self.generate_scene_summary(objects, targets)
        }

        return scene_desc

    def generate_scene_summary(self, objects, targets):
        """Generate a human-readable scene summary"""
        obj_classes = [obj['class'] for obj in objects]
        unique_classes = list(set(obj_classes))

        summary = {
            'detected_objects': len(objects),
            'unique_object_types': len(unique_classes),
            'object_types': unique_classes,
            'manipulation_candidates': len(targets),
            'dominant_objects': self.get_most_common_objects(objects)
        }

        return summary

    def get_most_common_objects(self, objects):
        """Get the most frequently detected object types"""
        class_counts = {}
        for obj in objects:
            cls = obj['class']
            class_counts[cls] = class_counts.get(cls, 0) + 1

        # Sort by count and return top 3
        sorted_classes = sorted(class_counts.items(), key=lambda x: x[1], reverse=True)
        return [cls for cls, count in sorted_classes[:3]]
```

## Testing and Validation

### Object Detection Test Suite

```python
import unittest
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

class TestObjectDetection(unittest.TestCase):
    def setUp(self):
        self.cv_bridge = CvBridge()
        self.test_image = self.create_test_image()
        self.test_detections = self.create_test_detections()

    def create_test_image(self):
        """Create a test image with known objects"""
        # Create a 640x480 RGB image with some colored rectangles
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Red rectangle (could be a "stop sign")
        img[100:200, 200:300] = [255, 0, 0]

        # Blue rectangle (could be a "bottle")
        img[250:350, 400:500] = [0, 0, 255]

        # Green rectangle (could be a "person")
        img[150:300, 100:150] = [0, 255, 0]

        return img

    def create_test_detections(self):
        """Create test detections"""
        detections = [
            {
                'class_id': 11,  # stop sign (red rectangle)
                'class_name': 'stop_sign',
                'confidence': 0.9,
                'bbox': [200, 100, 100, 100]  # x, y, width, height
            },
            {
                'class_id': 39,  # bottle (blue rectangle)
                'class_name': 'bottle',
                'confidence': 0.85,
                'bbox': [400, 250, 100, 100]
            }
        ]
        return detections

    def test_mock_detection(self):
        """Test the mock detection function"""
        detector = ObjectDetectionNode()
        results = detector.mock_detection(self.test_image)

        # We expect at least some detections
        self.assertGreater(len(results), 0)

        # Check that confidences are reasonable
        for det in results:
            self.assertGreaterEqual(det['confidence'], 0.0)
            self.assertLessEqual(det['confidence'], 1.0)

    def test_object_mapping(self):
        """Test object-action mapping"""
        mapper = ObjectActionMapper()

        # Test with a known object class
        plan = mapper.get_manipulation_plan('bottle', {'x': 100, 'y': 100})
        self.assertIsNotNone(plan)
        self.assertEqual(plan['object_class'], 'bottle')
        self.assertIn('grasp_type', plan)

    def test_scene_analysis(self):
        """Test scene analysis functionality"""
        analyzer = SceneAnalysisNode()

        # Test spatial relationship analysis
        test_objects = [
            {'class': 'bottle', 'confidence': 0.9, 'bbox_center': {'x': 100, 'y': 100}},
            {'class': 'cup', 'confidence': 0.8, 'bbox_center': {'x': 150, 'y': 100}},
            {'class': 'book', 'confidence': 0.7, 'bbox_center': {'x': 500, 'y': 400}}
        ]

        relationships = analyzer.analyze_spatial_relationships(test_objects)
        self.assertGreater(len(relationships), 0)

if __name__ == '__main__':
    unittest.main()
```

## Best Practices

1. **Lighting Conditions**: Ensure adequate lighting for reliable detection
2. **Camera Calibration**: Properly calibrate cameras for accurate 3D reconstruction
3. **Model Selection**: Choose appropriate models for your specific objects
4. **Performance**: Optimize for real-time performance requirements
5. **Robustness**: Handle various object orientations and occlusions
6. **Validation**: Test with diverse object sets and environmental conditions

## Integration with Capstone System

The object identification system integrates with the overall capstone project by:
- Detecting objects relevant to the current task
- Estimating 3D poses for manipulation planning
- Analyzing spatial relationships for navigation planning
- Providing scene understanding for decision making
- Filtering detections to focus on task-relevant objects

This object identification system enables the humanoid robot to perceive and understand its environment as part of the capstone project, supporting the overall goal of autonomous task execution based on voice commands.