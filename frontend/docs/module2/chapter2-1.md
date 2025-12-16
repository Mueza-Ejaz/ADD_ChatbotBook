---
sidebar_position: 1
title: Introduction to Gazebo & Digital Twins
---

# Introduction to Gazebo & Digital Twins

## Learning Objectives
- [ ] Understand the concept of digital twins in robotics.
- [ ] Set up and use Gazebo for robot simulation.
- [ ] Create simple virtual environments and robots in Gazebo.

## Theoretical Concepts
- What are Digital Twins?
- Importance of Simulation in Robotics
- Gazebo Architecture (World, Models, Plugins)
- SDF (Simulation Description Format)
- URDF (Unified Robot Description Format)

## Hands-on Examples

This section guides you through the initial steps of using Gazebo for robotic simulation and creating simple virtual environments.

### Installing Gazebo.

(Detailed steps for installing Gazebo, including dependencies and environment setup.)

### Launching Gazebo with an empty world.

(Commands and explanations for launching Gazebo with a blank simulation environment.)

### Spawning a simple robot model (e.g., a cube or sphere).

(Instructions for how to load and spawn a basic model into the Gazebo environment, potentially using the provided SDF example.)

### Basic control of a simulated robot.

(Explanation of how to interact with and control a simulated robot, possibly referencing the Python force script for programmatic control.)

## Code Snippets

### Example SDF for a simple model.

(Example `model.sdf` for a simple box)
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_box">
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.166667</ixx>
          <iyy>0.166667</iyy>
          <izz>0.166667</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Example URDF for a basic robot.

(Example `simple_robot.urdf` for a wheeled robot)
```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 -0.15 0.0" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Python script for basic Gazebo control (e.g., applying forces).

(Example `apply_force_script.py` for applying force to a model in Gazebo)
```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point
import sys

class GazeboForceApplier(Node):

    def __init__(self):
        super().__init__('gazebo_force_applier')
        self.cli = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ApplyBodyWrench.Request()

    def send_request(self):
        self.req.body_name = 'simple_box::link' # Adjust if using a different model/link
        self.req.reference_frame = 'world'
        
        self.req.wrench.force.x = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
        self.req.wrench.force.y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        self.req.wrench.force.z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        self.req.reference_point.x = 0.0
        self.req.reference_point.y = 0.0
        self.req.reference_point.z = 0.0
        
        self.req.start_time.sec = 0
        self.req.start_time.nanosec = 0
        self.req.duration.sec = 1
        self.req.duration.nanosec = 0

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    force_applier = GazeboForceApplier()
    response = force_applier.send_request()
    force_applier.get_logger().info(f'Apply Body Wrench response: {response.success}, {response.status_message}')
    force_applier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises & Assessments
- Create a custom Gazebo world with obstacles.
- Design a simple wheeled robot using URDF and simulate it.
- Implement a controller to make the wheeled robot move in a square.

## Further Reading
- Official Gazebo documentation.
- ROS 2 Gazebo integration tutorials.
