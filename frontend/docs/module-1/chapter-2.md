---
sidebar_position: 2
---

# Chapter 2: Nodes, Topics, Services in ROS 2

## Learning Objectives

*   Deep dive into the concepts of ROS 2 nodes, topics, and services.
*   Understand the lifecycle management of ROS 2 nodes.
*   Learn to create and manage ROS 2 communication patterns.

## Main Content

### Deep Dive into ROS 2 Nodes

In ROS 2, a node is the fundamental unit of computation, designed to be modular and perform a specific task. Each node is an independent executable process, allowing for distributed computing across multiple machines or even within a single machine.

#### Node Lifecycle Management

ROS 2 introduces a managed lifecycle for nodes, which is a significant improvement over ROS 1. This lifecycle provides a standardized way to transition nodes through different states, enhancing robustness and predictability in complex robotic systems.

The standard lifecycle states for a managed node are:

*   **Unconfigured:** Initial state after creation. No resources are allocated yet.
*   **Inactive:** Resources (like publishers, subscribers, services) are allocated, but not activated. The node is ready to become active.
*   **Active:** The node is fully operational and performing its primary function (e.g., publishing data, processing callbacks).
*   **Finalized:** The node is shutting down. Resources are deallocated.

Transitions between these states are explicit and can be triggered programmatically or via a lifecycle manager. This allows for controlled startup and shutdown sequences, which is critical for safety-critical applications or systems with complex interdependencies. For example, a robot's navigation stack might require its sensor drivers to be `Active` before it can transition to an `Active` state itself.

### Understanding ROS 2 Topics: Asynchronous Data Flow

Topics remain the backbone of real-time data streaming in ROS 2. They enable a many-to-many, publish-subscribe communication pattern, making them ideal for continuous data streams like sensor readings, odometry, and control commands.

#### Message Types

Every message exchanged over a topic has a defined type. These types are specified in `.msg` files, which are then compiled into language-specific data structures. Message types ensure that both publishers and subscribers agree on the format of the data being exchanged.

Common message types include:

*   `std_msgs`: Contains basic data types like `String`, `Int32`, `Float64`, etc.
*   `sensor_msgs`: For common sensor data, e.g., `Image`, `PointCloud2`, `Imu`.
*   `geometry_msgs`: For geometric primitives, e.g., `Point`, `Pose`, `Twist`.
*   `nav_msgs`: For navigation-related data, e.g., `Odometry`, `Path`.

Custom message types can also be defined by users, allowing for highly specialized data structures tailored to specific robot applications.

#### Quality of Service (QoS) Settings

QoS settings are a powerful feature in ROS 2, providing fine-grained control over the communication characteristics of topics, services, and actions. These settings allow developers to tune communication for specific application requirements, balancing factors like reliability, latency, and resource usage.

Key QoS policies include:

*   **Reliability:**
    *   `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`: Messages may be lost; suitable for high-frequency, non-critical data (e.g., video streams).
    *   `RMW_QOS_POLICY_RELIABILITY_RELIABLE`: Guarantees delivery of messages; suitable for critical data where no loss is acceptable (e.g., control commands).
*   **Durability:**
    *   `RMW_QOS_POLICY_DURABILITY_VOLATILE`: Only active subscribers receive messages; new subscribers do not receive historical data.
    *   `RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL`: The publisher will retain some samples and send them to new subscribers. Useful for publishing maps or configuration.
*   **History:**
    *   `RMW_QOS_POLICY_HISTORY_KEEP_LAST`: Only the last `depth` number of samples are stored.
    *   `RMW_QOS_POLICY_HISTORY_KEEP_ALL`: All samples are stored (up to resource limits).
*   **Depth:** Used with `KEEP_LAST` history policy, specifies the number of samples to keep.
*   **Liveliness:**
    *   `RMW_QOS_POLICY_LIVELINESS_AUTOMATIC`: Liveliness is asserted automatically by the RMW layer.
    *   `RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC`: Application must manually assert liveliness.
    *   `RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_PARTICIPANT`: Application must manually assert liveliness for the participant.

Choosing the correct QoS profile is crucial for optimizing performance and ensuring the correct behavior of your robotic system. For instance, a camera publishing high-frame-rate video might use `BEST_EFFORT` reliability to avoid latency, while a motor controller receiving command signals would use `RELIABLE` to ensure no commands are missed.

### ROS 2 Services: Synchronous Request/Response

Services provide a synchronous communication mechanism, where a client sends a request to a server and waits for a response. This is analogous to a function call in a distributed system.

#### Service Definition

Services are defined using `.srv` files, which specify both the request and response message structures. A `---` separates the request fields from the response fields.

Example:
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

#### Client and Server Implementation

*   **Service Server:** A node that implements the service logic. It waits for incoming requests, processes them, and sends back a response. The server registers itself with the ROS 2 graph, making its service discoverable by clients.
*   **Service Client:** A node that initiates a service call. It creates a client object for a specific service type and then calls the service, providing the request message. The client then blocks (or uses an asynchronous approach with futures) until it receives a response from the server.

Services are ideal for tasks such as:
*   Triggering an action (e.g., `take_picture`, `start_mission`).
*   Querying information (e.g., `get_map`, `get_robot_status`).
*   Configuring parameters (e.g., `set_speed_limit`).

### Creating and Managing ROS 2 Communication Patterns

The primary client libraries, `rclcpp` (C++) and `rclpy` (Python), provide the APIs to create and manage these communication patterns.

#### Example: Publisher-Subscriber (Python)

```python
# Publisher Node
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
        msg.data = 'Hello from ROS 2: %d' % self.i
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

```python
# Subscriber Node
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

These examples demonstrate the basic structure for creating ROS 2 nodes that communicate using topics. The `create_publisher` and `create_subscription` methods are used to set up the communication endpoints, specifying the message type, topic name, and a QoS history depth. The `timer_callback` for the publisher and `listener_callback` for the subscriber handle the actual message sending and receiving, respectively.

### Node Composition

ROS 2 encourages node composition, where multiple nodes can run within a single process. This reduces inter-process communication overhead and can improve performance, especially when nodes share common resources or frequently exchange large amounts of data. `rclcpp` provides mechanisms for composing nodes, allowing developers to choose between separate processes or composed processes based on performance and architectural needs.

### Conclusion

Nodes, topics, and services form the fundamental communication primitives in ROS 2. By understanding their roles, lifecycle management, message typing, and the critical impact of QoS settings, developers can design and implement robust, efficient, and scalable robotic systems. These concepts, underpinned by the flexible DDS layer, empower developers to build complex distributed applications with confidence.

## Code Examples

```python
# Placeholder for a simple ROS 2 subscriber node in Python
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

## Diagrams/Figures

*   **Figure 2.1: ROS 2 Node Lifecycle.** A state machine diagram illustrating the transitions between `Unconfigured`, `Inactive`, `Active`, and `Finalized` states, and the commands that trigger these transitions (e.g., `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`).
*   **Figure 2.2: ROS 2 Topic Data Flow.** Illustrates a publisher node sending messages to a topic and multiple subscriber nodes receiving messages from that topic, highlighting the asynchronous, one-to-many communication.
*   **Figure 2.3: ROS 2 Service Request/Response.** Illustrates a client node sending a request to a service server node, and the server processing the request and sending back a single response, highlighting the synchronous, one-to-one communication.
*   **Figure 2.4: Key QoS Policies for Topics.** An infographic or table showing the impact of different Reliability (Best Effort vs. Reliable) and Durability (Volatile vs. Transient Local) settings on message delivery and retention.

## Hands-on Exercises

1.  **Exercise 2.1: Implement a ROS 2 service client and server pair.**
    *   **Task:** Create a ROS 2 service server node (in C++ or Python) that offers a service to sum two integers. Create a ROS 2 service client node that calls this service with two numbers and prints the sum.
    *   **Verification:** Run both the server and client. Verify the client correctly receives and prints the sum.
2.  **Exercise 2.2: Experiment with different Quality of Service (QoS) settings for ROS 2 topics.**
    *   **Task:** Take your publisher and subscriber nodes from Chapter 1's exercises. Modify them to use different QoS profiles (e.g., `Reliable` vs. `Best Effort`, `Transient Local` vs. `Volatile` Durability). Observe the behavior, especially when subscribers start after publishers or when messages are intentionally dropped (e.g., by making the publisher run very fast and the subscriber slow).
    *   **Verification:** Document the observed differences in message delivery based on QoS settings.
3.  **Exercise 2.3: Explore ROS 2 Node Lifecycle Management.**
    *   **Task:** Create a simple lifecycle-managed ROS 2 node (in C++ or Python). Use the `ros2 lifecycle` command-line tools to transition the node through its different states (`configure`, `activate`, `deactivate`, `cleanup`, `shutdown`). Add print statements in your node's callbacks for each state transition to observe the process.
    *   **Verification:** Confirm that your node correctly transitions between states as commanded and that the corresponding callbacks are executed.

## Key Takeaways

*   ROS 2 communication is built upon nodes interacting via topics and services.
*   QoS settings are crucial for reliable and efficient data exchange.
