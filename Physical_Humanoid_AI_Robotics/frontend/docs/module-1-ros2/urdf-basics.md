---
sidebar_position: 4
title: 'URDF Basics for Humanoids'
---

# URDF Basics for Humanoids

This section covers Unified Robot Description Format (URDF) basics, with a focus on humanoid robot modeling. URDF is XML-based format used in ROS to describe robot models including kinematics, dynamics, and visual properties.

## Learning Objectives

After completing this section, you will be able to:
- Create basic URDF files for robot models
- Define robot kinematic chains using joints and links
- Specify visual and collision properties for humanoid robots
- Include inertial properties for dynamic simulation
- Use Xacro macros to simplify complex humanoid models

## URDF Fundamentals

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the robot's physical structure, including links, joints, and their relationships.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Child link connected via joint -->
  <link name="sensor_mount">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
```

## Humanoid Robot URDF Structure

Humanoid robots have a more complex structure with multiple limbs. Here's a simplified example:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Visual and Collision Properties

### Visual Elements
Visual elements define how the robot appears in simulation and visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Options: box, cylinder, sphere, mesh -->
    <box size="0.1 0.1 0.1"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Elements
Collision elements define the physical collision boundaries:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Properties
Inertial properties are crucial for dynamic simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

## Using Xacro for Complex Models

Xacro (XML Macros) simplifies complex URDF models by allowing parameterization and macros:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="link_length" value="0.3"/>
  <xacro:property name="link_radius" value="0.05"/>

  <!-- Macro for a simple arm segment -->
  <xacro:macro name="arm_segment" params="name parent xyz rpy">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${link_radius}" length="${link_length}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${link_radius}" length="${link_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
      </inertial>
    </link>

    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-2.0" upper="2.0" effort="10.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Use the macro to create arm segments -->
  <xacro:arm_segment name="upper_arm" parent="base_link"
                     xyz="0.1 0 0" rpy="0 0 0"/>
  <xacro:arm_segment name="lower_arm" parent="upper_arm"
                     xyz="${link_length} 0 0" rpy="0 0 0"/>
</robot>
```

## Humanoid-Specific Considerations

### Joint Limitations
Humanoid robots require careful attention to joint limits to mimic human-like motion:

```xml
<!-- Human-like shoulder joint with realistic limits -->
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <!-- Realistic human shoulder limits -->
  <limit lower="${-90 * M_PI / 180}" upper="${90 * M_PI / 180}"
         effort="50.0" velocity="2.0"/>
  <safety_controller k_position="20" k_velocity="400"
                     soft_lower_limit="${-85 * M_PI / 180}"
                     soft_upper_limit="${85 * M_PI / 180}"/>
</joint>
```

### Center of Mass
For stable humanoid locomotion, center of mass considerations are critical:

```xml
<!-- Simplified torso with proper mass distribution -->
<link name="torso">
  <inertial>
    <!-- Lower center of mass for stability -->
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <mass value="10.0"/>
    <!-- Higher moment of inertia for stability -->
    <inertia ixx="0.8" ixy="0.0" ixz="0.0" iyy="0.8" iyz="0.0" izz="0.4"/>
  </inertial>
</link>
```

## Lab Activity: Create a Simple Humanoid Model

Create a basic humanoid model with:
1. Torso, head, and two arms
2. Proper joint definitions with realistic limits
3. Visual and collision properties
4. Use Xacro macros to reduce code duplication

### Steps:
1. Create a new URDF file for your humanoid model
2. Define the basic structure with torso, head, and arms
3. Add appropriate joints with realistic limits
4. Include visual and collision properties
5. Use Xacro to simplify the model definition
6. Load the model in RViz to verify it displays correctly

### Expected Outcome:
- URDF file defines a valid humanoid robot structure
- Joints have appropriate types and limits
- Robot displays properly in RViz
- Xacro macros reduce redundancy in the model

## Best Practices

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Realistic Limits**: Use joint limits that reflect the physical capabilities
3. **Proper Inertias**: Calculate or estimate realistic inertial properties
4. **Xacro for Complex Models**: Use Xacro macros to manage complex humanoid models
5. **Validation**: Always validate URDF files using tools like `check_urdf`
6. **Documentation**: Comment complex URDF files to explain the structure

## Checklist

- [ ] Basic URDF structure understood and implemented
- [ ] Visual and collision properties defined correctly
- [ ] Inertial properties specified for dynamic simulation
- [ ] Xacro macros used to simplify model definition
- [ ] Humanoid-specific considerations applied
- [ ] Lab activity completed successfully

## Next Steps

In the next module, we'll explore digital twin technology using Gazebo and Unity for physics simulation and environment modeling.