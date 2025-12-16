# Chapter 2 Contract: The Robotic Nervous System (ROS 2)

**Chapter**: 2
**Directory**: `my-book/docs/chapter-2-ros2/`
**Prerequisites**: Chapter 1

---

## Deliverables

### Files to Create

| File | sidebar_position | Purpose |
|------|-----------------|---------|
| `_category_.json` | - | Category metadata |
| `index.md` | 1 | Chapter overview |
| `ros2-fundamentals.md` | 2 | ROS 2 architecture |
| `nodes-topics-services.md` | 3 | Communication patterns |
| `actions-and-rclpy.md` | 4 | Actions and Python API |
| `urdf-humanoids.md` | 5 | Robot description |

---

## Content Requirements

### index.md - Chapter Overview

**Must Include**:
- Learning objectives (understand ROS 2, create nodes, define URDF)
- Why ROS 2 is called "robotic nervous system"
- Prerequisites: Chapter 1, Python basics, Ubuntu 22.04
- Environment setup checklist

### ros2-fundamentals.md

**Must Include**:
- What is ROS 2 and why it exists
- DDS middleware explanation
- ROS 2 vs ROS 1 comparison
- Key concepts: workspace, packages, nodes
- Installation verification commands

**Code Examples**:
```bash
# Verify ROS 2 installation
ros2 --version
ros2 doctor --report
```

**Diagrams**:
- Mermaid: ROS 2 architecture layers (DDS, RMW, rclpy)

### nodes-topics-services.md

**Must Include**:
- Node concept and lifecycle
- Topics: publish/subscribe pattern
- Services: request/response pattern
- QoS (Quality of Service) basics
- When to use topics vs services

**Code Examples**:
1. Minimal publisher node (rclpy)
2. Minimal subscriber node (rclpy)
3. Simple service server and client
4. `ros2 topic` and `ros2 service` CLI usage

**Diagrams**:
- Mermaid: Pub/sub pattern
- Mermaid: Service request/response

### actions-and-rclpy.md

**Must Include**:
- Action concept (long-running tasks with feedback)
- Action server and client pattern
- rclpy API overview
- Executors and callbacks
- Best practices for node design

**Code Examples**:
1. Action server implementation
2. Action client with feedback handling
3. Multi-node example with timer

**Diagrams**:
- Mermaid: Action state machine

### urdf-humanoids.md

**Must Include**:
- What is URDF (Unified Robot Description Format)
- URDF elements: link, joint, visual, collision
- Joint types: revolute, prismatic, fixed
- Building a simple humanoid URDF
- Visualizing URDF in RViz2

**Code Examples**:
1. Simple two-link robot URDF
2. Humanoid torso and head URDF
3. Launch file to load URDF
4. RViz2 configuration

**Diagrams**:
- ASCII: URDF link/joint hierarchy
- Mermaid: Joint types and their motion

---

## Acceptance Criteria

- [ ] All 5 content files created with correct frontmatter
- [ ] `_category_.json` with label "2. ROS 2" and position 2
- [ ] DDS middleware explained clearly
- [ ] At least 8 code examples total across chapter
- [ ] All code examples include expected output
- [ ] Complete working URDF for simple humanoid
- [ ] rclpy patterns demonstrated for all communication types
- [ ] At least 4 Mermaid diagrams
- [ ] Build succeeds with no broken links
- [ ] Prerequisites admonition at chapter start
