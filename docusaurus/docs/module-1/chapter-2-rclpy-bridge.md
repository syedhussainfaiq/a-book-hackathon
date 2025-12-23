---
title: Chapter 2 - Bridging with rclpy
sidebar_position: 3
---

# Chapter 2: Bridging Python Agents to ROS Controllers

This chapter covers how to bridge Python-based AI agents to ROS controllers using rclpy, the Python client library for ROS 2. This integration enables AI-driven robot control by connecting intelligent agents to physical robotic systems.

## Learning Objectives

After completing this chapter, you will be able to:
- Install and set up rclpy for Python-ROS communication
- Create Python agents that communicate with ROS controllers
- Implement bidirectional communication between Python agents and ROS systems
- Apply best practices for efficient bridging

## Prerequisites

Before starting this chapter, you should have:
- Completed Chapter 1 on Nodes, Topics, and Services
- Basic understanding of Python programming
- Familiarity with AI/ML concepts
- Understanding of ROS 2 fundamentals

## Table of Contents

- [Introduction to rclpy](#introduction-to-rclpy)
- [Installation and Setup](#installation-and-setup)
- [Python Agent Integration](#python-agent-integration)
- [Bidirectional Communication](#bidirectional-communication)
- [Performance Considerations](#performance-considerations)
- [Troubleshooting](#troubleshooting)
- [Best Practices](#best-practices)
- [Summary](#summary)

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing Python APIs for creating ROS 2 nodes, publishers, subscribers, services, and other ROS 2 concepts. It allows Python-based AI agents to seamlessly integrate with ROS-based robotic systems.

### Key Features of rclpy

- Python bindings for ROS 2 client library (rcl)
- Support for all ROS 2 communication patterns (Nodes, Topics, Services, Actions)
- Integration with Python's asyncio for asynchronous operations
- Support for ROS 2 message and service definitions
- Compatibility with popular Python AI/ML libraries

### Why Bridge Python Agents to ROS?

Python is the dominant language in AI/ML development, with rich ecosystems like TensorFlow, PyTorch, and scikit-learn. ROS, on the other hand, is the standard middleware for robotics applications. Bridging these two enables:

- Application of advanced AI algorithms to robotic systems
- Rapid prototyping of intelligent robot behaviors
- Integration of machine learning models with physical robots
- Leveraging Python's data processing capabilities in robotics

## Installation and Setup

### Prerequisites

Before installing rclpy, ensure you have:

1. A working ROS 2 installation (e.g., Humble Hawksbill or Iron Irwini)
2. Python 3.8 or higher
3. pip package manager

### Installation Steps

1. **Install ROS 2**: Follow the official ROS 2 installation guide for your platform
2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash  # For Ubuntu with Humble
   ```
3. **Install rclpy** (usually included with ROS 2, but you can install separately if needed):
   ```bash
   pip install rclpy
   ```

### Verification

Create a simple test script to verify your installation:

```python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('test_node')
    node.get_logger().info('rclpy is working correctly!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Python Agent Integration

### Basic Integration Pattern

The integration of Python agents with ROS controllers typically follows this pattern:

1. The Python agent runs as a ROS node
2. The agent subscribes to sensor data from the robot
3. The agent processes the data using AI algorithms
4. The agent publishes control commands to the robot

### Example: Simple AI Controller

Let's create a Python agent that implements a simple AI controller for a mobile robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class SimpleAIController(Node):
    """
    A simple AI controller that navigates a robot using laser scan data
    """
    
    def __init__(self):
        super().__init__('simple_ai_controller')
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Robot state
        self.scan_data = None
        self.get_logger().info('Simple AI Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        # Filter out invalid readings (inf or nan)
        self.scan_data = self.scan_data[~np.isnan(self.scan_data)]
        self.scan_data = self.scan_data[self.scan_data != np.inf]

    def control_loop(self):
        """Main control loop"""
        if self.scan_data is None or len(self.scan_data) == 0:
            return
            
        # Simple AI logic: avoid obstacles
        min_distance = np.min(self.scan_data) if len(self.scan_data) > 0 else float('inf')
        
        cmd_vel = Twist()
        
        if min_distance < 1.0:  # If obstacle is closer than 1 meter
            # Turn to avoid obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5
        else:
            # Move forward
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
            
        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleAIController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Integration: ML Model Integration

Here's an example of integrating a machine learning model with ROS:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf  # Example with TensorFlow

class MLController(Node):
    """
    An AI controller that uses a machine learning model for navigation
    """
    
    def __init__(self):
        super().__init__('ml_controller')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/your/model.h5')
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Latest image
        self.latest_image = None
        self.get_logger().info('ML Controller initialized')

    def image_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv2.resize(cv_image, (224, 224))  # Resize for model input
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def control_loop(self):
        """Main control loop with ML model"""
        if self.latest_image is None:
            return
            
        # Preprocess image for model
        input_image = np.expand_dims(self.latest_image, axis=0).astype(np.float32) / 255.0
        
        # Run inference
        try:
            prediction = self.model.predict(input_image, verbose=0)
            
            # Interpret prediction (example: [linear_vel, angular_vel])
            cmd_vel = Twist()
            cmd_vel.linear.x = float(prediction[0][0])  # Example: first output is linear velocity
            cmd_vel.angular.z = float(prediction[0][1])  # Example: second output is angular velocity
            
            self.cmd_vel_publisher.publish(cmd_vel)
        except Exception as e:
            self.get_logger().error(f'Error during inference: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = MLController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bidirectional Communication

### Python-to-ROS Communication

Python agents can communicate with ROS systems through:

1. **Publishers**: Send data to ROS topics
2. **Service Clients**: Request services from ROS nodes
3. **Action Clients**: Initiate long-running tasks with feedback

### ROS-to-Python Communication

ROS systems can communicate with Python agents through:

1. **Subscribers**: Receive data from ROS topics
2. **Service Servers**: Provide services to other ROS nodes
3. **Action Servers**: Handle long-running tasks with feedback

### Example: Bidirectional Communication

Here's a more complex example showing bidirectional communication:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import time

class BidirectionalAgent(Node):
    """
    An agent that demonstrates bidirectional communication
    """
    
    def __init__(self):
        super().__init__('bidirectional_agent')
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'agent_status', 10)
        
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for commands from ROS
        self.command_subscriber = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )
        
        # Service server for AI capabilities
        self.ai_service = self.create_service(
            SetBool,
            'ai_decision',
            self.ai_decision_callback
        )
        
        # Timer for periodic status updates
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.agent_state = "IDLE"
        self.get_logger().info('Bidirectional Agent initialized')

    def command_callback(self, msg):
        """Handle commands from ROS system"""
        command = msg.data.lower()
        
        if command == "start":
            self.agent_state = "ACTIVE"
            self.get_logger().info('Agent activated')
            # Start some AI processing
            self.start_ai_processing()
        elif command == "stop":
            self.agent_state = "IDLE"
            self.get_logger().info('Agent deactivated')
        elif command == "move_forward":
            self.move_robot(0.5, 0.0)  # Move forward at 0.5 m/s
        elif command == "turn_left":
            self.move_robot(0.0, 0.5)  # Turn left at 0.5 rad/s

    def ai_decision_callback(self, request, response):
        """Handle AI decision service requests"""
        self.get_logger().info(f'AI decision requested: {request.data}')
        
        # Simulate some AI processing
        time.sleep(0.1)  # Simulate processing time
        
        # Make a simple decision based on agent state
        if self.agent_state == "ACTIVE":
            response.success = True
            response.message = "Decision made: Continue current task"
        else:
            response.success = False
            response.message = "Agent not active, cannot make decisions"
            
        return response

    def start_ai_processing(self):
        """Simulate starting AI processing"""
        self.get_logger().info('Starting AI processing...')
        # In a real implementation, this would start actual AI algorithms

    def move_robot(self, linear_vel, angular_vel):
        """Send velocity command to robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_publisher.publish(cmd_vel)

    def publish_status(self):
        """Publish agent status periodically"""
        status_msg = String()
        status_msg.data = f"Agent state: {self.agent_state}"
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    agent = BidirectionalAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Considerations

### Communication Overhead

When bridging Python agents to ROS controllers, consider:

- **Message frequency**: High-frequency messaging can create bottlenecks
- **Message size**: Large messages (e.g., images) require more processing
- **Serialization**: Converting data between Python and ROS formats takes time

### Best Practices for Performance

1. **Optimize message rates**: Only publish at necessary frequencies
2. **Use appropriate data types**: Choose efficient message formats
3. **Batch operations**: Combine multiple operations when possible
4. **Asynchronous processing**: Use async/await for non-blocking operations
5. **Threading**: Use threading for CPU-intensive AI processing

### Example: Optimized Message Handling

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from threading import Thread
import queue

class OptimizedAgent(Node):
    """
    An optimized agent that handles messages efficiently
    """
    
    def __init__(self):
        super().__init__('optimized_agent')
        
        # Queue for processing laser scans
        self.scan_queue = queue.Queue(maxsize=5)
        
        # Subscriber for laser scans
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Start processing thread
        self.processing_thread = Thread(target=self.process_scans, daemon=True)
        self.processing_thread.start()
        
        self.get_logger().info('Optimized Agent initialized')

    def scan_callback(self, msg):
        """Add scan to queue for processing"""
        try:
            # Add to queue, drop oldest if full
            if self.scan_queue.full():
                try:
                    self.scan_queue.get_nowait()  # Remove oldest
                except queue.Empty:
                    pass
            self.scan_queue.put_nowait(msg)
        except queue.Full:
            pass  # Queue is full, drop the scan

    def process_scans(self):
        """Process scans in separate thread"""
        while rclpy.ok():
            try:
                # Get scan from queue with timeout
                scan_msg = self.scan_queue.get(timeout=1.0)
                
                # Process scan data
                scan_data = np.array(scan_msg.ranges)
                scan_data = scan_data[~np.isnan(scan_data)]
                scan_data = scan_data[scan_data != np.inf]
                
                # Perform AI processing here
                self.perform_ai_processing(scan_data)
                
                self.scan_queue.task_done()
            except queue.Empty:
                continue  # No scans to process, continue loop
            except Exception as e:
                self.get_logger().error(f'Error processing scan: {e}')

    def perform_ai_processing(self, scan_data):
        """Perform AI processing on scan data"""
        # Implement your AI algorithm here
        self.get_logger().info(f'Processed scan with {len(scan_data)} readings')

def main(args=None):
    rclpy.init(args=args)
    agent = OptimizedAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

Common issues when bridging Python agents to ROS controllers:

1. **Import errors**: Ensure ROS environment is sourced before running Python scripts
2. **Node name conflicts**: Use unique node names across your system
3. **Message type mismatches**: Verify that publishers and subscribers use compatible message types
4. **Threading issues**: Be careful when using threading with rclpy; use async/await when possible
5. **Performance bottlenecks**: Monitor message rates and processing times

## Best Practices

1. **Error handling**: Always implement proper error handling for robust operation
2. **Resource management**: Properly destroy nodes and clean up resources
3. **Logging**: Use appropriate logging levels for debugging and monitoring
4. **Configuration**: Use ROS parameters for configurable behavior
5. **Testing**: Test integration thoroughly in simulation before real hardware

## Summary

In this chapter, we explored how to bridge Python agents to ROS controllers using rclpy:

- We covered the installation and setup of rclpy
- We implemented examples of Python agents communicating with ROS systems
- We demonstrated bidirectional communication patterns
- We discussed performance considerations and optimization techniques
- We provided troubleshooting tips and best practices

This integration enables the application of advanced AI algorithms to robotic systems, bridging the gap between AI development in Python and robotics control in ROS. In the next chapter, we'll cover URDF (Unified Robot Description Format) for modeling humanoid robots.