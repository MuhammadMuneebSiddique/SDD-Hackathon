---
sidebar_position: 2
title: 'Nodes, Topics, and Services in ROS 2'
---

# Nodes, Topics, and Services in ROS 2

This section covers the fundamental communication patterns in ROS 2: nodes, topics, and services. These form the backbone of the robotic nervous system.

## Learning Objectives

After completing this section, you will be able to:
- Create and manage ROS 2 nodes
- Implement publisher-subscriber communication using topics
- Set up request-response communication using services
- Understand the differences between topics and services

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

### Creating a Node

In ROS 2, nodes are typically created as classes that inherit from `rclpy.Node`. Here's a basic example:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal_node')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

Topics enable asynchronous, many-to-many communication between nodes using a publish-subscribe pattern.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services

Services enable synchronous, request-response communication between nodes.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Key Differences

| Feature | Topics | Services |
|---------|--------|----------|
| Communication Style | Publish-Subscribe (async) | Request-Response (sync) |
| Connection | Many-to-many | One-to-one |
| Message Delivery | Best-effort, may be lost | Guaranteed delivery |
| Latency | Lower | Higher due to request-response |
| Use Case | Continuous data streams | Action requests, computations |

## Lab Activity: Simple Communication

Create a simple ROS 2 system with:
1. A publisher node that sends a message every second
2. A subscriber node that receives and logs the messages
3. A service server that performs a simple calculation
4. A service client that calls the server

### Steps:
1. Create a new ROS 2 package: `ros2 pkg create --build-type ament_python my_robot_communication`
2. Implement the nodes as shown in the examples above
3. Test the communication between nodes
4. Verify that topics and services work as expected

### Expected Outcome:
- Publisher sends messages at regular intervals
- Subscriber receives and logs all messages
- Service client successfully calls the server and receives results

## Checklist

- [ ] Node creation and lifecycle management understood
- [ ] Topic publisher-subscriber pattern implemented
- [ ] Service request-response pattern implemented
- [ ] Differences between topics and services understood
- [ ] Lab activity completed successfully

## Next Steps

In the next section, we'll explore how to bridge ROS 2 with AI agents using the rclpy interface.