---
title: Chapter 3 - URDF for Humanoids
sidebar_position: 4
---

# Chapter 3: URDF for Humanoids

This chapter covers the Unified Robot Description Format (URDF) for modeling humanoid robots. URDF is an XML-based format that describes robot models, including their physical and visual properties, which is essential for simulation and visualization in ROS-based robotic systems.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the structure and syntax of URDF for humanoid robots
- Create URDF models for basic humanoid robots
- Define joints, links, and materials in URDF
- Visualize URDF models in ROS tools
- Apply best practices for efficient URDF modeling

## Prerequisites

Before starting this chapter, you should have:
- Completed Chapters 1 and 2 on ROS 2 fundamentals and rclpy bridging
- Basic understanding of XML syntax
- Familiarity with 3D coordinate systems and transformations
- Understanding of robotics kinematics concepts

## Table of Contents

- [Introduction to URDF](#introduction-to-urdf)
- [URDF Syntax and Structure](#urdf-syntax-and-structure)
- [Links and Joints for Humanoids](#links-and-joints-for-humanoids)
- [Visual and Collision Properties](#visual-and-collision-properties)
- [Materials and Physical Properties](#materials-and-physical-properties)
- [Creating a Humanoid URDF Model](#creating-a-humanoid-urdf-model)
- [Visualization in ROS](#visualization-in-ros)
- [Troubleshooting](#troubleshooting)
- [Best Practices](#best-practices)
- [Summary](#summary)

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial parameters, visual appearance, and collision properties.

### Why URDF for Humanoid Robots?

Humanoid robots have complex kinematic structures with multiple degrees of freedom. URDF provides a standardized way to:

- Define the robot's physical structure
- Specify joint limits and properties
- Describe visual appearance for simulation
- Enable kinematic calculations
- Support motion planning and control

### URDF in the ROS Ecosystem

URDF integrates with various ROS tools:
- **RViz**: For visualization
- **Gazebo**: For physics simulation
- **MoveIt!**: For motion planning
- **TF**: For coordinate transformations

## URDF Syntax and Structure

A URDF file is an XML document with the following basic structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links definition -->
  <link name="link_name">
    <!-- Link properties -->
  </link>

  <!-- Joints definition -->
  <joint name="joint_name" type="joint_type">
    <!-- Joint properties -->
  </joint>

  <!-- Materials definition -->
  <material name="material_name">
    <!-- Material properties -->
  </material>
</robot>
```

### Key Elements

1. **robot**: The root element containing the entire robot description
2. **link**: Represents a rigid part of the robot
3. **joint**: Defines the connection between two links
4. **material**: Defines visual appearance properties

## Links and Joints for Humanoids

### Links

Links represent rigid parts of the robot. For humanoid robots, common links include:

- **torso/base**: The main body of the robot
- **head**: The head component
- **arms**: Upper arm, lower arm, hand
- **legs**: Upper leg, lower leg, foot
- **etc.**: Any other rigid components

Each link can have:
- **visual**: How the link appears visually
- **collision**: How the link interacts in collision detection
- **inertial**: Physical properties for simulation

### Joints

Joints define how links connect and move relative to each other. For humanoid robots, common joint types include:

- **revolute**: Rotational joint with limits (like human joints)
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint
- **fixed**: No movement between links
- **floating**: 6 DOF joint (rarely used)
- **planar**: Movement in a plane (rarely used)

### Example: Basic Link Structure

```xml
<link name="upper_arm">
  <inertial>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.2" radius="0.05"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.2" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

### Example: Basic Joint Structure

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.15 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Visual and Collision Properties

### Visual Properties

Visual elements define how the robot appears in visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Choose one geometry type -->
    <box size="0.1 0.1 0.1"/>                    <!-- Box shape -->
    <cylinder radius="0.05" length="0.2"/>        <!-- Cylinder shape -->
    <sphere radius="0.05"/>                       <!-- Sphere shape -->
    <mesh filename="package://my_robot/meshes/link.dae" scale="1 1 1"/>  <!-- Mesh file -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Properties

Collision elements define how the robot interacts in collision detection:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Similar to visual but often simpler shapes for efficiency -->
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

## Materials and Physical Properties

### Materials

Materials define the visual appearance of links:

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 0.5"/>  <!-- Last value is alpha for transparency -->
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>
```

### Inertial Properties

Inertial properties define the physical characteristics for simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

## Creating a Humanoid URDF Model

Let's create a simplified humanoid robot model with a torso, head, two arms, and two legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <!-- Base/Torso -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left, mirrored) -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Right Leg (similar to left, mirrored) -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <link name="right_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

</robot>
```

## Visualization in ROS

### Using RViz

To visualize your URDF model in RViz:

1. Launch RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. Add a RobotModel display:
   - Click "Add" in the Displays panel
   - Select "RobotModel" under "By display type"
   - Set the "Robot Description" parameter to the name of your parameter (usually "robot_description")

3. Load the URDF to the parameter server:
   ```bash
   ros2 param set /robot_state_publisher robot_description --string-type "$(cat path/to/your/robot.urdf)"
   ```

### Using joint_state_publisher_gui

To visualize joint movement:

```bash
# First, run robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..." # your URDF content

# Then run joint_state_publisher_gui to control joints
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Checking URDF validity

Before using your URDF, validate it:

```bash
# Check if URDF is well-formed
check_urdf path/to/your/robot.urdf

# Or visualize the kinematic chain
urdf_to_graphiz path/to/your/robot.urdf
```

## Troubleshooting

Common issues when working with URDF models:

1. **XML syntax errors**: Check for proper closing tags and attribute formatting
2. **Missing parent/child links**: Ensure all joints reference existing links
3. **Inertia calculation**: Use proper formulas for different shapes (spheres, cylinders, boxes)
4. **Joint limits**: Set appropriate limits based on real robot capabilities
5. **Origin transformations**: Verify xyz and rpy values are correct
6. **Mass and inertia values**: Use realistic values for simulation stability

## Best Practices

1. **Use consistent naming**: Follow a clear convention for link and joint names
2. **Start simple**: Begin with a basic model and add complexity gradually
3. **Validate regularly**: Use URDF validation tools frequently during development
4. **Use xacro for complex models**: Xacro allows macros and parameterization
5. **Proper inertial properties**: Calculate realistic mass and inertia values
6. **Consider simulation performance**: Use simpler collision geometries when possible
7. **Document your model**: Include comments explaining complex parts of your URDF

## Summary

In this chapter, we covered the fundamentals of URDF for modeling humanoid robots:

- We learned the structure and syntax of URDF files
- We explored how to define links and joints for humanoid robots
- We covered visual and collision properties
- We created a complete humanoid URDF model with torso, head, arms, and legs
- We discussed visualization techniques in ROS tools
- We provided troubleshooting tips and best practices

URDF is essential for representing humanoid robots in ROS-based systems, enabling simulation, visualization, and control. With this knowledge, you can create detailed models of humanoid robots for various applications.