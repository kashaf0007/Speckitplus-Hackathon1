# Chapter 4 Contract: The AI-Robot Brain (NVIDIA Isaac)

**Chapter**: 4
**Directory**: `my-book/docs/chapter-4-isaac/`
**Prerequisites**: Chapter 3

---

## Deliverables

### Files to Create

| File | sidebar_position | Purpose |
|------|-----------------|---------|
| `_category_.json` | - | Category metadata |
| `index.md` | 1 | Chapter overview |
| `isaac-sim-intro.md` | 2 | Isaac Sim fundamentals |
| `synthetic-data.md` | 3 | Data generation |
| `isaac-ros.md` | 4 | GPU-accelerated ROS |
| `nav2-bipedal.md` | 5 | Navigation for humanoids |

---

## Content Requirements

### index.md - Chapter Overview

**Must Include**:
- Learning objectives (Isaac Sim, synthetic data, VSLAM, Nav2)
- What makes Isaac different from Gazebo
- When to use Isaac vs Gazebo
- Prerequisites: Chapter 3, NVIDIA RTX GPU
- Warning about hardware requirements with Gazebo fallback

### isaac-sim-intro.md

**Must Include**:
- NVIDIA Omniverse platform overview
- Isaac Sim architecture
- Installation and setup (standalone and Docker)
- USD (Universal Scene Description) basics
- Loading robot assets
- Running first simulation

**Code Examples**:
1. Isaac Sim launch command
2. Python scripting in Isaac Sim
3. Loading URDF as USD
4. Basic simulation control script

**Diagrams**:
- Mermaid: Isaac Sim architecture
- Mermaid: Omniverse ecosystem

**Fallback**: Gazebo equivalent commands for each section

### synthetic-data.md

**Must Include**:
- Why synthetic data for perception
- Domain randomization concept
- Replicator overview
- Generating labeled images
- Camera and lighting variation
- Exporting datasets (COCO, KITTI formats)

**Code Examples**:
1. Replicator basic randomization script
2. Camera placement and variation
3. Semantic segmentation setup
4. Dataset export configuration

**Diagrams**:
- Mermaid: Synthetic data pipeline

**Fallback**: Manual data collection in Gazebo

### isaac-ros.md

**Must Include**:
- Isaac ROS packages overview
- GPU-accelerated perception nodes
- cuVSLAM for Visual SLAM
- DNN inference nodes
- Integration with standard ROS 2 nodes

**Code Examples**:
1. Isaac ROS Docker setup
2. cuVSLAM launch configuration
3. DNN inference node example
4. Perception pipeline launch file

**Diagrams**:
- Mermaid: Isaac ROS perception stack
- Table: Isaac ROS vs standard ROS 2 packages

**Fallback**: ORB-SLAM3 as alternative VSLAM

### nav2-bipedal.md

**Must Include**:
- Nav2 architecture overview
- Costmap configuration for humanoids
- Bipedal locomotion considerations
- Conservative velocity limits
- Path planning for narrow spaces
- Recovery behaviors for humanoids

**Code Examples**:
1. Nav2 parameters YAML for humanoid
2. Costmap configuration
3. Launch file for Nav2 with Isaac Sim
4. Custom behavior tree example

**Diagrams**:
- Mermaid: Nav2 architecture
- ASCII: Costmap layers

**Fallback**: Nav2 with Gazebo simulation

---

## Acceptance Criteria

- [ ] All 5 content files created with correct frontmatter
- [ ] `_category_.json` with label "4. NVIDIA Isaac" and position 4
- [ ] Isaac Sim 2023.1+ features documented
- [ ] At least 12 code examples total across chapter
- [ ] Gazebo fallback provided for each Isaac-specific feature
- [ ] Synthetic data pipeline fully explained
- [ ] Visual SLAM (cuVSLAM) configuration provided
- [ ] Nav2 configured for bipedal robot constraints
- [ ] At least 5 Mermaid diagrams
- [ ] Build succeeds with no broken links
- [ ] GPU requirement warning prominently displayed
