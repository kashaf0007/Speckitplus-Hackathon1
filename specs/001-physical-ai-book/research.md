# Research: Physical AI & Humanoid Robotics Book

**Date**: 2025-12-17
**Feature**: 001-physical-ai-book
**Purpose**: Resolve technical unknowns and establish best practices for book content

---

## 1. ROS 2 Humble Best Practices

### Decision: Use ROS 2 Humble Hawksbill (LTS)

**Rationale**:
- Humble is the current LTS release (supported until May 2027)
- Widest community adoption and documentation
- Stable API suitable for educational content
- Iron Irwini as secondary option for newer features

**Alternatives Considered**:
- ROS 2 Iron: Newer but shorter support window
- ROS 2 Rolling: Too unstable for educational content
- ROS 1 Noetic: Legacy, not recommended for new projects

### Key API Patterns for Book

```python
# Standard rclpy node pattern
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        # Publisher, subscriber, service, action setup
```

**Documentation Reference**: https://docs.ros.org/en/humble/Tutorials.html

---

## 2. Gazebo Simulation Platform

### Decision: Use Gazebo Sim (Ignition Fortress/Garden)

**Rationale**:
- Gazebo Classic is deprecated (end of life 2025)
- Gazebo Sim (formerly Ignition) is the successor
- Better physics engine options (DART recommended)
- Native ROS 2 integration via ros_gz packages

**Alternatives Considered**:
- Gazebo Classic: Deprecated, avoid for new content
- PyBullet: Good for RL but limited sensor simulation
- MuJoCo: Excellent physics but less ROS integration

### Physics Engine Selection

| Engine | Use Case | Performance |
|--------|----------|-------------|
| DART | Humanoid articulated bodies | Best for bipedal |
| ODE | General purpose, legacy | Good compatibility |
| Bullet | Collision-heavy scenarios | Fast but less accurate |

**Documentation Reference**: https://gazebosim.org/docs/fortress

---

## 3. Unity ROS Integration

### Decision: Use Unity Robotics Hub with ROS-TCP-Connector

**Rationale**:
- Official Unity-maintained solution
- Supports ROS 2 natively
- Good for high-fidelity visualization and HRI scenarios
- Active development and community

**Setup Pattern**:
1. Install Unity Robotics Hub package
2. Configure ROS-TCP-Endpoint on ROS 2 side
3. Import URDF via URDF Importer package
4. Use ROS-TCP-Connector for message passing

**Alternatives Considered**:
- ROS#: C# implementation, less maintained
- ROS2 for Unity: Experimental, not production-ready
- Unreal Engine: Steeper learning curve

**Documentation Reference**: https://github.com/Unity-Technologies/Unity-Robotics-Hub

---

## 4. NVIDIA Isaac Sim Integration

### Decision: Use Isaac Sim 2023.1+ with Isaac ROS

**Rationale**:
- Industry-standard for photorealistic simulation
- Native synthetic data generation
- GPU-accelerated perception pipelines
- Direct Nav2 integration

### Key Components

| Component | Purpose | Chapter |
|-----------|---------|---------|
| Isaac Sim | Simulation environment | Ch4 |
| Isaac ROS | GPU-accelerated nodes | Ch4 |
| Replicator | Synthetic data generation | Ch4 |
| cuVSLAM | Visual SLAM | Ch4 |

### Fallback for Non-GPU Users

Provide Gazebo Sim alternatives for each Isaac Sim example:
- Use Gazebo for basic simulation
- Document CPU-based perception alternatives
- Note performance differences

**Documentation Reference**: https://developer.nvidia.com/isaac-sim

---

## 5. Speech-to-Text Integration

### Decision: Use OpenAI Whisper (API and local)

**Rationale**:
- State-of-the-art accuracy
- Available as API and local model
- Python integration straightforward
- Supports multiple languages

### Implementation Pattern

```python
# API-based (recommended for quick start)
import openai
audio_file = open("command.mp3", "rb")
transcript = openai.Audio.transcribe("whisper-1", audio_file)

# Local model (for offline/privacy)
import whisper
model = whisper.load_model("base")
result = model.transcribe("command.mp3")
```

**Alternatives Considered**:
- Google Speech-to-Text: Good but requires GCP setup
- Azure Speech: Similar quality, Microsoft ecosystem
- Vosk: Offline-first, lower accuracy

**Documentation Reference**: https://platform.openai.com/docs/guides/speech-to-text

---

## 6. LLM Integration for Robotics

### Decision: Use structured prompting with function calling

**Rationale**:
- Function calling enables reliable action extraction
- Structured output for ROS 2 action generation
- Works with GPT-4, Claude, and open models

### Cognitive Planning Pattern

```python
# LLM task decomposition
system_prompt = """
You are a robot task planner. Given a natural language command,
decompose it into a sequence of robot actions.
Output JSON with: {"actions": [{"type": "navigate", "target": "kitchen"}, ...]}
"""

# Function calling schema
functions = [{
    "name": "execute_robot_action",
    "parameters": {
        "type": "object",
        "properties": {
            "action_type": {"enum": ["navigate", "pick", "place", "speak"]},
            "target": {"type": "string"}
        }
    }
}]
```

**Alternatives Considered**:
- Direct text parsing: Fragile, not recommended
- Custom fine-tuned models: Too complex for book scope
- Rule-based planning: Limited flexibility

**Local Model Options**:
- Llama 2/3: Good performance, runs on consumer hardware
- Mistral: Efficient, good for edge deployment
- Ollama: Easy local model management

---

## 7. Nav2 for Bipedal Navigation

### Decision: Use Nav2 with custom locomotion plugins

**Rationale**:
- Standard ROS 2 navigation stack
- Modular architecture allows bipedal adaptations
- Well-documented with tutorials
- Isaac ROS provides acceleration

### Bipedal Considerations

| Challenge | Solution |
|-----------|----------|
| Balance dynamics | Use conservative velocity limits |
| Foot placement | Custom local planner plugin |
| Stair navigation | Out of scope (flat terrain only) |
| Recovery behaviors | Adapt for humanoid kinematics |

**Documentation Reference**: https://navigation.ros.org/

---

## 8. URDF for Humanoid Robots

### Decision: Use simplified humanoid URDF with visual meshes

**Rationale**:
- Full humanoid URDF is complex (50+ joints)
- Book uses simplified model for learning
- Reference real URDFs (PR2, Pepper) for complexity

### Simplified Structure

```xml
<!-- Humanoid URDF structure -->
<robot name="simple_humanoid">
  <link name="base_link"/>
  <link name="torso"/>
  <link name="head"/>
  <link name="left_arm"/> <!-- simplified as single link -->
  <link name="right_arm"/>
  <link name="left_leg"/>
  <link name="right_leg"/>
  <!-- Joints connecting links -->
</robot>
```

**Progressive Complexity**:
1. Chapter 2: Basic torso + head
2. Chapter 3: Add limbs with physics
3. Chapter 4: Full articulated model in Isaac

---

## 9. Docusaurus Configuration

### Decision: Use Docusaurus 3.x with docs-only mode

**Rationale**:
- Already installed in my-book directory
- Excellent for technical documentation
- Built-in versioning, search, dark mode
- MDX support for interactive examples

### Key Configuration

```typescript
// docusaurus.config.ts
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A simulation-first approach to embodied AI',
  // Enable mermaid for diagrams
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
};
```

**Documentation Reference**: https://docusaurus.io/docs

---

## 10. Diagram Strategy

### Decision: Use Mermaid for all diagrams

**Rationale**:
- Native Docusaurus support
- Version-controllable (text-based)
- No external image generation needed
- Supports flowcharts, sequence diagrams, architecture

### Example Patterns

```mermaid
flowchart LR
    A[Voice Input] --> B[Whisper ASR]
    B --> C[LLM Planner]
    C --> D[ROS 2 Actions]
    D --> E[Robot Execution]
```

**Fallback**: ASCII art for simple diagrams in code blocks

---

## Summary of Decisions

| Topic | Decision | Risk Level |
|-------|----------|------------|
| ROS 2 Version | Humble LTS | Low |
| Simulation | Gazebo Sim + Isaac Sim | Medium |
| Unity Integration | ROS-TCP-Connector | Low |
| Speech-to-Text | OpenAI Whisper | Low |
| LLM Integration | Function calling pattern | Low |
| Navigation | Nav2 with adaptations | Medium |
| Diagrams | Mermaid | Low |
| URDF | Simplified progressive | Low |

All technical unknowns resolved. Ready for Phase 1 design.
