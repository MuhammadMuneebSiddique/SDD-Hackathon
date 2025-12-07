---
sidebar_position: 3
title: 'ROS 2 Python (rclpy) Bridge for AI Agents'
---

# ROS 2 Python (rclpy) Bridge for AI Agents

This section covers how to use the rclpy library to bridge AI agents with ROS 2 systems, enabling AI algorithms to control and interact with robotic platforms.

## Learning Objectives

After completing this section, you will be able to:
- Interface AI agents with ROS 2 using rclpy
- Implement message passing between AI systems and robots
- Create custom message types for AI-robot communication
- Design AI control loops that interact with ROS 2 nodes

## Introduction to rclpy

The `rclpy` library is the Python client library for ROS 2, providing a Python API for ROS concepts such as nodes, publishers, subscribers, services, and parameters.

### Basic AI Agent Integration

Here's how to create an AI agent that interfaces with ROS 2:

```python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribers for sensor data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        # Internal state for the AI agent
        self.sensor_data = None
        self.get_logger().info('AI Agent Node initialized')

    def lidar_callback(self, msg):
        """Process LIDAR data for the AI agent"""
        self.sensor_data = np.array(msg.ranges)
        # Filter out invalid readings
        self.sensor_data = self.sensor_data[~np.isnan(self.sensor_data)]
        self.sensor_data = self.sensor_data[self.sensor_data > 0]

    def ai_decision_callback(self):
        """Main AI decision making loop"""
        if self.sensor_data is not None:
            # Simple obstacle avoidance algorithm
            cmd_vel = self.simple_navigation_ai(self.sensor_data)
            self.cmd_vel_publisher.publish(cmd_vel)

    def simple_navigation_ai(self, sensor_data):
        """Simple AI algorithm for navigation"""
        msg = Twist()

        # Calculate minimum distance in front of robot
        front_distances = sensor_data[len(sensor_data)//2-10:len(sensor_data)//2+10]
        min_front_dist = np.min(front_distances) if len(front_distances) > 0 else float('inf')

        if min_front_dist < 1.0:  # Obstacle too close
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Turn right
        else:
            msg.linear.x = 0.5  # Move forward
            msg.angular.z = 0.0

        return msg
```

## Custom Message Types for AI Communication

For more complex AI-robot interactions, you may need custom message types:

```python
# Custom AI command message (define in msg/AICommand.msg)
# string action_type
# float32[] parameters
# string target_object
```

### Using Custom Messages

```python
from my_robot_msgs.msg import AICommand  # Custom message type

class AdvancedAIAgent(Node):
    def __init__(self):
        super().__init__('advanced_ai_agent')

        self.ai_command_publisher = self.create_publisher(
            AICommand,
            '/ai_commands',
            10
        )

        self.ai_command_subscriber = self.create_subscription(
            AICommand,
            '/ai_feedback',
            self.ai_feedback_callback,
            10
        )

    def ai_feedback_callback(self, msg):
        """Process feedback from the robot to the AI"""
        self.get_logger().info(f'AI received feedback: {msg.action_type}')
```

## Integration with AI Frameworks

### TensorFlow/PyTorch Integration

```python
import tensorflow as tf
# or
import torch

class MLAgentNode(Node):
    def __init__(self):
        super().__init__('ml_agent_node')

        # Load pre-trained model
        self.model = self.load_model()

        self.sensor_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.process_sensor_data,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def load_model(self):
        """Load your trained ML model"""
        # Load model from file or create new one
        return tf.keras.models.load_model('path/to/model')
        # or return torch.load('path/to/model.pt')

    def process_sensor_data(self, msg):
        """Process sensor data through ML model"""
        sensor_array = np.array(msg.ranges)
        # Preprocess for model
        input_data = self.preprocess(sensor_array)
        # Get prediction from model
        prediction = self.model.predict(input_data)
        # Convert prediction to robot command
        cmd_vel = self.prediction_to_command(prediction)
        self.cmd_publisher.publish(cmd_vel)
```

## State Management for AI Agents

AI agents often need to maintain state across multiple decision cycles:

```python
from std_msgs.msg import String
import json

class StatefulAIAgent(Node):
    def __init__(self):
        super().__init__('stateful_ai_agent')

        self.ai_state_publisher = self.create_publisher(
            String,
            '/ai_state',
            10
        )

        self.state = {
            'current_task': 'idle',
            'task_progress': 0.0,
            'last_action': 'none',
            'belief_state': {}
        }

        self.timer = self.create_timer(0.5, self.publish_state)

    def update_state(self, task, progress, action, belief_state=None):
        """Update the internal state of the AI agent"""
        self.state['current_task'] = task
        self.state['task_progress'] = progress
        self.state['last_action'] = action
        if belief_state:
            self.state['belief_state'].update(belief_state)

    def publish_state(self):
        """Publish current AI state for monitoring"""
        state_msg = String()
        state_msg.data = json.dumps(self.state)
        self.ai_state_publisher.publish(state_msg)
```

## Lab Activity: AI-ROS Integration

Create an AI agent that performs simple navigation using sensor data:

### Steps:
1. Create a new ROS 2 package for the AI agent
2. Implement an AI node that subscribes to sensor data (LIDAR, camera, etc.)
3. Create a simple AI algorithm that processes sensor data and outputs commands
4. Test the AI agent in simulation

### Expected Outcome:
- AI agent successfully receives sensor data
- AI agent makes decisions based on sensor input
- Robot follows AI-generated commands
- AI state is properly maintained across cycles

## Best Practices

1. **Separation of Concerns**: Keep AI logic separate from ROS communication
2. **Error Handling**: Implement robust error handling for sensor failures
3. **Timing**: Ensure AI decision frequency matches robot control requirements
4. **State Management**: Properly maintain AI state across multiple cycles
5. **Safety**: Implement safety checks to prevent dangerous robot behavior

## Checklist

- [ ] AI agent successfully interfaces with ROS 2 using rclpy
- [ ] Custom message types created and used appropriately
- [ ] ML framework integration implemented
- [ ] State management for AI agent implemented
- [ ] Lab activity completed successfully

## Next Steps

In the next section, we'll explore URDF basics for humanoid robots, which will help us define the physical structure of robots that our AI agents will control.