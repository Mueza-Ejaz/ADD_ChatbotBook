---
sidebar_position: 1
---

# Chapter 1: ROS 2 Architecture & Core Concepts

## Learning Objectives

*   Understand the fundamental architecture of ROS 2.
*   Grasp core ROS 2 concepts like nodes, topics, and services.
*   Learn how to set up a basic ROS 2 workspace.

## Main Content

### Introduction to ROS 2 Architecture

ROS 2 (Robot Operating System 2) represents a significant evolution from its predecessor, ROS 1, primarily driven by the need for enhanced real-time performance, support for multiple middleware implementations, and improved security. At its core, ROS 2 is a flexible framework for writing robot software. It's not an operating system in the traditional sense, but rather a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot applications.

The architecture of ROS 2 is fundamentally distributed and highly modular, designed to operate across a variety of computing environments, from embedded systems to cloud infrastructure. Its key architectural differentiator is its reliance on a Data Distribution Service (DDS) layer, which handles the communication between different components of a robotic system.

### Core ROS 2 Concepts

To understand ROS 2, it's crucial to grasp its fundamental building blocks:

#### 1. Nodes: The Computational Units

A **node** is an executable process that performs computation. In ROS 2, the philosophy is to design small, modular, and single-purpose nodes. For example, a robot might have one node for controlling motors, another for reading sensor data, and a third for path planning. This modularity enhances fault tolerance, as the failure of one node does not necessarily bring down the entire system. Nodes are typically written in C++ (using `rclcpp`) or Python (using `rclpy`).

#### 2. Topics: Asynchronous Data Streaming

**Topics** are named buses over which nodes exchange messages asynchronously. This is the primary means of data streaming in ROS 2. A node can _publish_ messages to a topic, and other nodes can _subscribe_ to that topic to receive those messages. Messages are strongly typed (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`), ensuring data consistency. The communication is one-to-many: a single publisher can send data to multiple subscribers on the same topic. Topics are ideal for continuous data streams like sensor readings, joint states, or video feeds.

#### 3. Services: Synchronous Request/Response

**Services** enable synchronous request/response communication between nodes. Unlike topics, which are asynchronous and one-way, services involve a _client_ node sending a request to a _server_ node and waiting for a response. This is particularly useful for operations that require an immediate result, such as asking a robot arm to move to a specific position and waiting for confirmation of completion, or querying a map server for an occupancy grid.

#### 4. Actions: Long-Running Tasks with Feedback

**Actions** are designed for long-running, goal-oriented tasks that require periodic feedback and the ability to be preempted. They extend the request/response pattern of services by adding:
*   **Goal:** The client sends a goal to the action server (e.g., "drive 10 meters forward").
*   **Feedback:** The action server sends continuous feedback to the client (e.g., "robot has driven 2 meters," "robot has driven 5 meters").
*   **Result:** Once the goal is completed (or aborted), the action server sends a final result (e.g., "goal achieved," "path blocked").
Actions are suitable for tasks like navigation, complex manipulation, or sequence execution.

#### 5. Parameters: Dynamic Configuration

**Parameters** are dynamic configuration values for nodes. They allow users to change a node's behavior without recompiling the code. Nodes can declare parameters with default values, and these values can be read and set at runtime. This is useful for adjusting sensor thresholds, PID gains, or other configurable aspects of a robot's operation.

#### 6. ROS 2 Bags: Data Recording and Playback

**ROS 2 Bags** are a mechanism for recording and playing back ROS 2 message data. They are invaluable for debugging, data analysis, and developing algorithms offline. You can record data from topics, services, and actions, and then play it back later as if the robot were operating live.

### The DDS Layer: The Heart of ROS 2 Communication

A fundamental architectural change in ROS 2 is the abstraction of the communication layer using the Data Distribution Service (DDS) standard. DDS is an open international standard for publish-subscribe communication for real-time systems.

Key aspects of the DDS layer in ROS 2:

*   **Middleware Agnostic:** ROS 2 is not tied to a single DDS implementation. It supports various DDS vendors (e.g., Fast RTPS, Connext, OpenSplice), allowing users to choose the one best suited for their application's needs regarding performance, reliability, and licensing.
*   **Decentralized Communication:** Unlike ROS 1's `roscore`, which acted as a central point of coordination, ROS 2's DDS communication is decentralized. Nodes discover each other dynamically over the network. This removes a single point of failure and improves scalability and robustness.
*   **Quality of Service (QoS) Policies:** DDS provides a rich set of QoS policies that allow developers fine-grained control over communication behavior. These policies dictate aspects such as:
    *   **Reliability:** Guarantees delivery of messages (or best effort).
    *   **Durability:** Whether historical messages are sent to new subscribers.
    *   **Liveliness:** How publishers and subscribers detect each other's presence.
    *   **History:** How many messages are kept in the queue.
    *   **Deadline:** The expected maximum period between samples.
    *   **Latency Budget:** The maximum acceptable delay for message propagation.
    These QoS settings are crucial for tailoring communication to specific application requirements, especially in real-time robotics.
*   **Discovery:** DDS handles the automatic discovery of publishers and subscribers on the network. When a new node starts, it announces itself and can discover other nodes publishing or subscribing to relevant topics/services.

### Client Libraries

ROS 2 provides client libraries that allow developers to write nodes in various programming languages. The two primary client libraries are:

*   **rclcpp (C++)**: The C++ client library, built on top of `rcl` (ROS Client Library) which provides a C API to the underlying DDS implementation. `rclcpp` offers an object-oriented interface for creating nodes, publishers, subscribers, services, and actions in C++. It's generally preferred for performance-critical components.
*   **rclpy (Python)**: The Python client library, also built on `rcl`. `rclpy` provides a Pythonic interface for interacting with ROS 2. It is often favored for rapid prototyping, high-level control logic, and applications where development speed is more critical than raw performance.

Both `rclcpp` and `rclpy` expose similar functionalities, allowing seamless integration of components written in different languages within the same ROS 2 system.

### Overall Communication Mechanisms

The communication in ROS 2 fundamentally revolves around DDS, facilitated by the `rcl` (ROS Client Library) layer and exposed through language-specific client libraries (`rclcpp`, `rclpy`).

When a node publishes a message to a topic:
1.  The client library (`rclcpp` or `rclpy`) serializes the message.
2.  `rcl` passes the serialized data to the configured DDS middleware.
3.  The DDS middleware handles the network transport, discovery, and QoS enforcement.
4.  Subscribing nodes, also using their respective client libraries and `rcl`, receive the data from DDS.
5.  `rcl` deserializes the data.
6.  The client library then provides the deserialized message to the subscriber's callback function.

This layered approach ensures interoperability, flexibility, and robust communication for distributed robotic systems. The abstraction provided by DDS allows ROS 2 developers to focus on application logic rather than low-level networking details, while still offering powerful control through QoS policies.

### Setting Up a Basic ROS 2 Workspace

A ROS 2 workspace is a directory structure where you can organize, build, and install your ROS 2 packages.

**Steps:**

1.  **Create a Workspace Directory:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  **Initialize the Workspace (Optional, for older versions or specific setups):**
    In most modern ROS 2 distributions, `colcon build` handles initialization automatically.
3.  **Clone/Create ROS 2 Packages:** Place your package source code (e.g., a package containing your nodes) into the `src` directory.
    ```bash
    # Example: create a new package
    ros2 pkg create --build-type ament_cmake my_first_ros2_package
    # Or clone an existing one
    # git clone <your_repo_url> src/my_package
    ```
4.  **Build the Workspace:** Use `colcon build` to compile the packages in your workspace.
    ```bash
    colcon build
    ```
    This command will build all packages in the `src` directory and install the executables and libraries into the `install` directory within your workspace.
5.  **Source the Setup Files:** Before running any ROS 2 executables from your workspace, you must source the `setup.bash` (or `setup.ps1` for Windows, `setup.zsh` for Zsh) file from your workspace's `install` directory. This adds your workspace's packages to your ROS 2 environment.
    ```bash
    . install/setup.bash
    ```
    It's common to add this sourcing command to your `~/.bashrc` (or equivalent shell configuration file) so it's automatically sourced in new terminals.

By following these steps, you create a functional environment to develop and run your ROS 2 applications.

## Code Examples

```cpp
// Placeholder for a simple ROS 2 publisher node in C++
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  std_msgs::msg::String message;
  message.data = "Hello, ROS 2!";
  rclcpp::WallRate loop_rate(1.0); // 1 Hz
  while (rclcpp::ok()) {
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
```

## Diagrams/Figures

*   **Figure 1.1: ROS 2 High-Level Architecture.** A block diagram showing the interaction between User Applications, Client Libraries (rclcpp, rclpy), ROS Client Library (rcl), RMW (ROS Middleware), and DDS implementations. Illustrate how user code interacts through client libraries, which then use `rcl` and RMW to communicate via DDS.
*   **Figure 1.2: ROS 2 Communication Patterns.** A visual representation of nodes, topics, and services.
    *   For **Topics**: Show Node A publishing to a topic, and Node B and Node C subscribing to the same topic, illustrating one-to-many asynchronous communication.
    *   For **Services**: Show Client Node making a request to Server Node, and Server Node providing a response, illustrating synchronous request-response.
    *   For **Actions**: Show a Client sending a Goal, an Action Server providing Feedback, and finally a Result, indicating a long-running, interruptible task.
*   **Figure 1.3: ROS 2 DDS Quality of Service (QoS) Policies.** A table or infographic summarizing key QoS policies (Reliability, Durability, History, Liveliness, Deadline, Latency Budget) with brief explanations and typical use cases.

## Hands-on Exercises

1.  **Exercise 1.1: Set up a new ROS 2 workspace and compile a publisher.**
    *   **Task:** Create a new ROS 2 workspace, navigate into it, and create a C++ or Python package. Copy the provided publisher code example into your new package. Build the workspace using `colcon build`.
    *   **Verification:** Source your workspace and run the publisher node. Observe it printing "Publishing: Hello, ROS 2!"
2.  **Exercise 1.2: Create a subscriber node to receive messages.**
    *   **Task:** In the same workspace, create another C++ or Python package. Implement a simple subscriber node that listens to the `topic` that your publisher from Exercise 1.1 is publishing to. The subscriber should print the received messages to the console.
    *   **Verification:** Run both the publisher and subscriber nodes simultaneously. Confirm that the subscriber node correctly receives and prints the "Hello, ROS 2!" messages.
3.  **Exercise 1.3: Experiment with ROS 2 `ros2 topic` commands.**
    *   **Task:** While your publisher and subscriber nodes are running, open a new terminal (and remember to source your ROS 2 environment and workspace). Use `ros2 topic list` to see active topics, `ros2 topic info /topic` to get information about your topic, and `ros2 topic echo /topic` to monitor the messages being published.
    *   **Verification:** Confirm that you can see your `/topic` and its messages using these commands.

## Key Takeaways

*   **ROS 2 Architecture:** Fundamentally distributed, modular, and built on a Data Distribution Service (DDS) layer for communication.
*   **Core Concepts:** Nodes (computational units), Topics (asynchronous data streams), Services (synchronous request/response), Actions (long-running tasks with feedback), and Parameters (dynamic configuration).
*   **DDS Layer:** Enables decentralized communication, middleware agnosticism, and fine-grained control over communication behavior via Quality of Service (QoS) policies.
*   **Client Libraries:** `rclcpp` (C++) and `rclpy` (Python) provide the APIs for developing ROS 2 applications.
*   **Workspace Setup:** A structured environment for organizing, building, and installing ROS 2 packages, crucial for development.
