# Chapter 3 Contract: Digital Twin Simulation (Gazebo & Unity)

**Chapter**: 3
**Directory**: `my-book/docs/chapter-3-simulation/`
**Prerequisites**: Chapter 2

---

## Deliverables

### Files to Create

| File | sidebar_position | Purpose |
|------|-----------------|---------|
| `_category_.json` | - | Category metadata |
| `index.md` | 1 | Chapter overview |
| `gazebo-physics.md` | 2 | Gazebo simulation |
| `unity-simulation.md` | 3 | Unity integration |
| `sensor-simulation.md` | 4 | Sensor models |

---

## Content Requirements

### index.md - Chapter Overview

**Must Include**:
- Learning objectives (Gazebo setup, Unity ROS, sensor simulation)
- What is a digital twin
- Why simulation before hardware
- Prerequisites: Chapter 2, ROS 2 Humble installed
- Software requirements (Gazebo Sim, Unity)

### gazebo-physics.md

**Must Include**:
- Gazebo Classic vs Gazebo Sim (Ignition) - use Gazebo Sim
- Installation and setup
- Physics engines: ODE, DART, Bullet
- World file structure (SDF format)
- Spawning robot models
- Physics parameters: gravity, friction, contacts

**Code Examples**:
1. Basic world SDF file
2. Spawning URDF in Gazebo via ros_gz
3. Launch file for Gazebo with ROS 2
4. Physics parameter tuning example

**Diagrams**:
- Mermaid: Gazebo-ROS 2 architecture
- Table: Physics engine comparison

### unity-simulation.md

**Must Include**:
- Why Unity for robotics (HRI, visualization)
- Unity Robotics Hub overview
- ROS-TCP-Connector setup
- URDF Importer usage
- Basic scene with robot
- Message passing between Unity and ROS 2

**Code Examples**:
1. ROS-TCP-Endpoint launch
2. Unity C# subscriber example
3. Unity C# publisher example
4. URDF import configuration

**Diagrams**:
- Mermaid: Unity-ROS 2 communication flow

### sensor-simulation.md

**Must Include**:
- Sensor types in robotics (proprioceptive, exteroceptive)
- LiDAR simulation in Gazebo
- Depth camera (RGB-D) simulation
- IMU simulation
- Sensor noise models
- Visualizing sensor data in RViz2

**Code Examples**:
1. LiDAR sensor plugin configuration (SDF)
2. Depth camera plugin configuration
3. IMU plugin configuration
4. RViz2 sensor visualization setup
5. Reading sensor topics in Python

**Diagrams**:
- Mermaid: Sensor data flow to ROS 2
- ASCII: LiDAR scan pattern

---

## Acceptance Criteria

- [ ] All 4 content files created with correct frontmatter
- [ ] `_category_.json` with label "3. Simulation" and position 3
- [ ] Gazebo Sim (not Classic) used throughout
- [ ] At least 10 code examples total across chapter
- [ ] Working launch file to spawn humanoid in Gazebo
- [ ] Unity-ROS 2 bidirectional communication demonstrated
- [ ] All three sensor types (LiDAR, camera, IMU) covered
- [ ] At least 3 Mermaid diagrams
- [ ] Build succeeds with no broken links
- [ ] Warning admonition about Gazebo Classic deprecation
