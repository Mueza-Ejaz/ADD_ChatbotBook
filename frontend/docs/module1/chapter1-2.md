---
sidebar_position: 2
title: Advanced ROS 2 Concepts
---

# Advanced ROS 2 Concepts

## Learning Objectives
- [ ] Understand ROS 2 custom message creation.
- [ ] Explore ROS 2 actions and services.
- [ ] Implement advanced communication patterns in ROS 2.

## Theoretical Concepts
- Custom Message Definition (`.msg` files)
- ROS 2 Services (request-response communication)
- ROS 2 Actions (long-running tasks with feedback)
- Parameter management
- Launch Files for complex systems

## Hands-on Examples

This section provides hands-on guidance for implementing advanced ROS 2 communication patterns.

### Creating and using custom messages.

(Detailed steps for defining a custom `.msg` file, building the package, and using the custom message in Python nodes.)

### Implementing a simple ROS 2 service server and client.

(Detailed steps for creating a service server and client, along with explanations of the provided code snippets.)

### Implementing a simple ROS 2 action server and client.

(Detailed steps for creating an action server and client, along with explanations of the provided code snippets.)

### Using launch files to start multiple nodes.

(Detailed steps for creating and running a Python launch file to orchestrate multiple ROS 2 nodes.)

## Code Snippets

### Custom message definition.

(Example `.msg` file content: `Num.msg`)
```
int64 num
```

### Service server and client in Python.

(Example `add_two_ints_server.py`)
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a} b={request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
```

(Example `add_two_ints_client.py`)
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {minimal_client.req.a} + {minimal_client.req.b} = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action server and client in Python.

(Example `fibonacci_action_server.py`)
```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

(Example `fibonacci_action_client.py`)
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

### Example ROS 2 launch file.

(Example `my_robot_launch.py`)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypackage',
            executable='minimal_publisher',
            name='talker',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='mypackage',
            executable='minimal_subscriber',
            name='listener',
            output='screen',
            emulate_tty=True,
        ),
    ])
```

## Exercises & Assessments
- Design a custom message for a robot's status.
- Create a service to trigger a robot's movement.
- Implement an action for a robot to navigate to a goal.

## Further Reading
- ROS 2 Foxy/Humble documentation on custom interfaces.
- Tutorials on ROS 2 launch system.
