---
title: Chapter 1 - Nodes, Topics, and Services
sidebar_position: 2
---

# Chapter 1: Nodes, Topics, and Services

This chapter introduces the fundamental communication patterns in ROS 2: Nodes, Topics, and Services. These components form the backbone of any ROS-based robotic system.

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the purpose and function of Nodes in ROS 2
- Describe how Topics enable publisher-subscriber communication
- Understand the role of Services in request-reply communication
- Create simple Nodes that communicate via Topics and Services

## Prerequisites

Before starting this chapter, you should have:
- Completed the Module 1 introduction
- Basic understanding of Python programming
- Familiarity with command-line interfaces

## Table of Contents

- [Introduction to ROS 2 Architecture](#introduction-to-ros-2-architecture)
- [Nodes: The Building Blocks](#nodes-the-building-blocks)
- [Topics: Publisher-Subscriber Communication](#topics-publisher-subscriber-communication)
- [Services: Request-Reply Communication](#services-request-reply-communication)
- [Practical Examples](#practical-examples)
- [Troubleshooting](#troubleshooting)
- [Summary](#summary)

## Introduction to ROS 2 Architecture

ROS 2 (Robot Operating System 2) is not an actual operating system but rather a collection of tools, libraries, and conventions that provide a framework for building robotic applications. It enables distributed computation across multiple processes and devices, making it easier to develop complex robotic systems.

The core architectural elements of ROS 2 are:

1. **Nodes**: Processes that perform computation
2. **Topics**: Named buses over which nodes exchange messages
3. **Services**: Synchronous request/reply communication between nodes

These elements work together to enable flexible and modular robot software design.

## Nodes: The Building Blocks

Nodes are the fundamental building blocks of any ROS 2 system. They are individual processes that perform specific computations and communicate with other nodes in the system.

### Characteristics of Nodes

- Each node runs a specific task or function
- Nodes are designed to be modular and reusable
- Multiple nodes can run simultaneously
- Nodes can be written in different programming languages
- Nodes communicate with each other via Topics and Services

### Creating a Node

In Python, nodes are created by subclassing the `Node` class from the `rclpy` library:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example creates a node that publishes "Hello World" messages to a topic at a regular interval.

## Topics: Publisher-Subscriber Communication

Topics enable asynchronous communication between nodes using a publisher-subscriber pattern. Publishers send messages to a topic, and subscribers receive messages from that topic.

### Key Concepts

- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Message**: The data structure sent between nodes
- **Topic Name**: A unique identifier for the communication channel

### Topic Communication Example

Here's an example of a subscriber that receives messages from the publisher we created above:

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
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Reply Communication

Services provide synchronous request-reply communication between nodes. A client sends a request to a service, and the service sends back a response.

### Key Concepts

- **Service Server**: A node that provides a service
- **Service Client**: A node that calls a service
- **Service Type**: Defines the request and response message types
- **Service Name**: A unique identifier for the service

### Service Example

Here's an example of a simple service that adds two integers:

Service Definition (`AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

Service Server:
```python
from add_two_ints.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Service Client:
```python
from add_two_ints.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Examples

Let's walk through a practical example that demonstrates all three concepts working together in a simple robotic scenario.

### Scenario: Temperature Monitoring System

Imagine a simple robot that monitors temperature in different rooms:

1. **Temperature Sensor Node**: A node that publishes temperature readings to a topic
2. **Data Logger Node**: A node that subscribes to temperature readings and logs them
3. **Emergency Service**: A service that can be called to trigger emergency protocols

#### Temperature Sensor Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensor(Node):

    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 2  # Publish every 2 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading (between 18 and 25 degrees Celsius)
        msg.data = 18.0 + random.random() * 7.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    temp_sensor = TemperatureSensor()
    
    try:
        rclpy.spin(temp_sensor)
    except KeyboardInterrupt:
        pass
    finally:
        temp_sensor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Data Logger Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Data logger started, listening for temperature data...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received temperature: {msg.data:.2f}°C')
        # In a real application, you would log this to a file or database

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    
    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

Common issues when working with Nodes, Topics, and Services:

1. **Nodes not communicating**: Check that nodes are on the same ROS domain ID
2. **Topic names don't match**: Ensure publisher and subscriber use identical topic names
3. **Service not found**: Verify the service name and that the server is running
4. **Message type mismatch**: Confirm that publishers, subscribers, and services use compatible message types

## Summary

In this chapter, we covered the fundamental communication patterns in ROS 2:

- **Nodes** are the basic computational units that perform specific tasks
- **Topics** enable asynchronous publisher-subscriber communication
- **Services** provide synchronous request-reply communication

These concepts form the foundation of ROS 2 architecture and are essential for building distributed robotic systems. In the next chapter, we'll explore how to bridge Python agents to ROS controllers using rclpy.