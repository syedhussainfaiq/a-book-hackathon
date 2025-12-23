---
title: Chapter 1 - Gazebo Physics Simulation
sidebar_position: 6
---

# Chapter 1: Gazebo Physics Simulation

This chapter introduces physics simulation in Gazebo for digital twin applications. You'll learn how to create and configure physics simulations that accurately represent real-world physics for humanoid robots.

## Learning Objectives

After completing this chapter, you will be able to:
- Create and configure Gazebo world files for physics simulation
- Set up robot models with proper physics parameters
- Configure physics parameters (gravity, collisions, dynamics) for realistic simulation
- Validate physics simulation behavior

## Prerequisites

Before starting this chapter, you should have:
- Completed Module 1 (ROS 2 fundamentals)
- Basic understanding of physics concepts (gravity, collisions, dynamics)
- Familiarity with humanoid robot systems

## Table of Contents

- [Introduction to Gazebo Physics](#introduction-to-gazebo-physics)
- [World File Creation and Configuration](#world-file-creation-and-configuration)
- [Model Configuration for Humanoid Robots](#model-configuration-for-humanoid-robots)
- [Physics Parameters](#physics-parameters)
- [Practical Examples](#practical-examples)
- [Troubleshooting](#troubleshooting)
- [Summary](#summary)

## Introduction to Gazebo Physics

Gazebo is a physics-based simulation engine that provides accurate and efficient simulation of robotic systems. It includes:
- High-fidelity physics simulation with multiple physics engines
- Realistic rendering capabilities
- Sensor simulation
- Plugin architecture for custom functionality

For digital twin applications, Gazebo serves as the physics foundation that enables accurate simulation of real-world interactions.

### Key Concepts

1. **World Files**: XML-based files that define the simulation environment
2. **Models**: Robot and object definitions with physical properties
3. **Physics Engine**: The underlying system that calculates physics interactions
4. **Sensors**: Virtual sensors that simulate real-world sensor data

## World File Creation and Configuration

World files define the simulation environment in Gazebo. They specify the physics parameters, lighting, objects, and models that make up the simulation.

### Basic World File Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Physics parameters -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -1.0</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sky -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
  </world>
</sdf>
```

### Physics Parameters in World Files

The `<physics>` tag in world files defines the physics simulation parameters:

- **gravity**: Sets the gravity vector (x, y, z) in m/s²
- **max_step_size**: Maximum time step for the physics engine in seconds
- **real_time_factor**: Target real-time factor (1.0 means real-time)
- **real_time_update_rate**: Update rate in Hz

### Creating a World File for Humanoid Robots

Here's an example world file configured for humanoid robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Physics parameters optimized for humanoid simulation -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1.0</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Optional: Add objects for interaction -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.5 0.8</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
  </world>
</sdf>
```

## Model Configuration for Humanoid Robots

Models define the physical properties of robots and objects in Gazebo. For humanoid robots, models need to accurately represent the physical characteristics and kinematic structure.

### Basic Model Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_humanoid">
    <!-- Links represent rigid bodies -->
    <link name="base_link">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <!-- Additional links for limbs -->
    <link name="head">
      <pose>0 0 0.3 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <!-- Joint connecting links -->
    <joint name="neck_joint" type="revolute">
      <parent>base_link</parent>
      <child>head</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
      <pose>0 0 0.3 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

### Physics Parameters for Humanoid Models

When configuring humanoid models, pay special attention to:

1. **Mass**: Accurate mass distribution for each link
2. **Inertia**: Proper moment of inertia tensors for realistic physics
3. **Collision geometry**: Simplified but accurate collision shapes
4. **Joint limits**: Realistic range of motion for each joint

## Physics Parameters

Physics parameters control the behavior and accuracy of the simulation. Understanding these parameters is crucial for creating realistic simulations.

### Gravity Configuration

Gravity is typically set to Earth's gravity (9.8 m/s²) but can be adjusted for different environments:

```xml
<gravity>0 0 -9.8</gravity>  <!-- Earth gravity -->
<gravity>0 0 -1.62</gravity> <!-- Moon gravity -->
<gravity>0 0 -3.7</gravity>  <!-- Mars gravity -->
```

### Time Step Configuration

The time step affects simulation accuracy and performance:

- **max_step_size**: Smaller values increase accuracy but decrease performance
- **real_time_update_rate**: Higher values provide more accurate simulation but require more computation
- **real_time_factor**: Set to 1.0 for real-time simulation

### Solver Parameters

The physics solver parameters affect the stability and accuracy of the simulation:

- **iters**: Number of iterations for the solver (higher = more stable but slower)
- **sor**: Successive over-relaxation parameter (typically 1.2-1.5)
- **cfm**: Constraint force mixing (typically 0 for stable constraints)
- **erp**: Error reduction parameter (0.1-0.8 for stability)

## Practical Examples

### Example 1: Simple Humanoid in a Room

Let's create a simple simulation with a humanoid robot in a room:

**World file (humanoid_room.world):**
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_room">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Room walls -->
    <model name="wall_1">
      <pose>5 0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 3</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1.0</direction>
    </light>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
  </world>
</sdf>
```

### Example 2: Physics Parameter Tuning

For humanoid robots, physics parameters need to be carefully tuned to ensure stable simulation:

```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Troubleshooting

Common issues with Gazebo physics simulation:

1. **Robot falls through the ground**: Check collision geometries and physics parameters
2. **Unstable simulation**: Increase solver iterations or reduce time step
3. **Robot parts flying apart**: Verify joint constraints and link masses
4. **Simulation runs too slowly**: Reduce physics update rate or simplify collision geometries

### Debugging Tips

- Use Gazebo's built-in visualization tools to inspect collision geometries
- Monitor the simulation's real-time factor to ensure performance
- Check model masses and inertias for realistic values
- Verify that joints have appropriate limits and types

## Summary

In this chapter, we covered the fundamentals of Gazebo physics simulation for digital twin applications:

- We learned how to create and configure world files with appropriate physics parameters
- We explored model configuration for humanoid robots with accurate physical properties
- We examined key physics parameters that affect simulation behavior
- We provided practical examples and troubleshooting tips

These concepts form the foundation for creating realistic physics simulations in your digital twin environment. In the next chapter, we'll explore how to use Unity for high-fidelity rendering and human-robot interaction.