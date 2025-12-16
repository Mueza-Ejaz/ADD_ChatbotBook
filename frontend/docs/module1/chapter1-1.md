---
sidebar_position: 1
title: Introduction to ROS 2
---

# Introduction to ROS 2

## Learning Objectives
- [ ] Understand the core concepts of ROS 2.
- [ ] Set up a basic ROS 2 workspace.
- [ ] Write and run a simple ROS 2 publisher and subscriber.

## Theoretical Concepts
- What is ROS 2?
- ROS 2 Architecture (Nodes, Topics, Services, Actions, Parameters)
- ROS 2 vs ROS 1
- ROS 2 Development Tools (RCLPY, RCLC)

## Hands-on Examples

This section guides you through setting up a basic ROS 2 environment and implementing your first publisher and subscriber nodes.

### Setting up ROS 2 Environment

(Detailed steps for installing ROS 2, sourcing setup files, etc. will go here)

### Creating a ROS 2 Package

(Detailed steps for creating a new ROS 2 Python package will go here)

### Implementing a 'Hello World' Publisher

Below is the Python code for a simple ROS 2 publisher. This node will publish "Hello World" messages to a topic named 'topic'.

(Description of how to save, build, and run the publisher node)

### Implementing a 'Hello World' Subscriber

Here is the Python code for a simple ROS 2 subscriber. This node will listen to the 'topic' and print any messages it receives.

(Description of how to save, build, and run the subscriber node)

## Code Snippets

### Python code for publisher.

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

### Python code for subscriber.

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

## Exercises & Assessments
- Modify the 'Hello World' example to send custom messages.
- Create a ROS 2 node that echoes received messages.

## Further Reading
- Official ROS 2 documentation.
- Relevant academic papers.
