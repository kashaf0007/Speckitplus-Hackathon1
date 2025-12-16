# Quickstart: Physical AI & Humanoid Robotics Book

**Date**: 2025-12-17
**Feature**: 001-physical-ai-book
**Purpose**: Implementation guide for content authors

---

## Prerequisites

### Development Environment

```bash
# Verify Node.js (18+)
node --version

# Navigate to book directory
cd my-book

# Install dependencies (if not done)
npm install

# Start development server
npm run start
```

### Content Tools

- Markdown editor (VS Code recommended)
- Mermaid preview extension
- Git for version control

---

## Quick Implementation Steps

### 1. Configure Docusaurus (First)

Update `my-book/docusaurus.config.ts`:
- Change title to "Physical AI & Humanoid Robotics"
- Update tagline
- Configure navbar with chapter links
- Enable Mermaid for diagrams

### 2. Create Chapter Directories

```bash
cd my-book/docs

# Create all chapter directories
mkdir -p chapter-1-introduction
mkdir -p chapter-2-ros2
mkdir -p chapter-3-simulation
mkdir -p chapter-4-isaac
mkdir -p chapter-5-vla
mkdir -p chapter-6-capstone
```

### 3. Add Category Files

Each chapter needs `_category_.json`:

```json
{
  "label": "1. Introduction to Physical AI",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Learn the foundations of Physical AI and embodied intelligence."
  }
}
```

### 4. Create Content Pages

Each page follows this template:

```markdown
---
sidebar_position: 1
title: "Page Title"
description: "SEO description for this page"
---

# Page Title

Introduction paragraph...

## Section 1

Content with [internal links](/docs/chapter-1-introduction/).

:::tip Pro Tip
Use admonitions for important callouts.
:::

### Code Example

\`\`\`python title="example.py"
# Your code here
\`\`\`

## Summary

What the reader learned...

## Next Steps

Link to next page or chapter.
```

---

## Content Creation Workflow

### For Each Chapter

1. **Create `_category_.json`** with label and position
2. **Create `index.md`** with chapter overview
3. **Create content pages** in sidebar_position order
4. **Add code examples** with language identifiers
5. **Include diagrams** using Mermaid
6. **Add admonitions** for tips/warnings
7. **Verify build** with `npm run build`

### Quality Checklist (Per Page)

- [ ] Frontmatter complete (title, description, sidebar_position)
- [ ] Introduction explains page purpose
- [ ] At least one code example with expected output
- [ ] Internal links use correct paths
- [ ] Admonitions used appropriately
- [ ] No spelling/grammar errors
- [ ] Build succeeds after adding page

---

## Code Example Standards

### Python (rclpy)

```python title="minimal_publisher.py"
#!/usr/bin/env python3
"""Minimal ROS 2 publisher example."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [minimal_publisher]: Publishing: 'Hello, ROS 2!'
```

### YAML (Configuration)

```yaml title="robot_config.yaml"
robot:
  name: simple_humanoid
  joints:
    - name: head_pan
      type: revolute
      limit: [-1.57, 1.57]
```

### Bash (Commands)

```bash title="Build and run"
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build package
colcon build --packages-select my_package

# Run node
ros2 run my_package my_node
```

---

## Diagram Standards

### Flowchart (Mermaid)

```markdown
\`\`\`mermaid
flowchart LR
    A[Input] --> B[Process]
    B --> C[Output]
\`\`\`
```

### Architecture Diagram

```markdown
\`\`\`mermaid
graph TB
    subgraph ROS2[ROS 2 System]
        N1[Perception Node]
        N2[Planning Node]
        N3[Control Node]
    end
    N1 --> N2
    N2 --> N3
\`\`\`
```

### Sequence Diagram

```markdown
\`\`\`mermaid
sequenceDiagram
    User->>Whisper: Voice Command
    Whisper->>LLM: Transcript
    LLM->>ROS2: Action Plan
    ROS2->>Robot: Execute
\`\`\`
```

---

## Common Patterns

### Prerequisites Box

```markdown
:::info Prerequisites
- ROS 2 Humble installed
- Python 3.10+
- Completed Chapter 2
:::
```

### Warning Box

```markdown
:::warning
NVIDIA Isaac Sim requires an RTX GPU. See [Gazebo alternative](#gazebo-fallback) if unavailable.
:::
```

### Tip Box

```markdown
:::tip
Use `ros2 topic echo /topic_name` to debug message flow.
:::
```

---

## Build and Verify

```bash
# Development server (hot reload)
npm run start

# Production build (validates all links)
npm run build

# Serve production build locally
npm run serve
```

### Common Build Errors

| Error | Cause | Fix |
|-------|-------|-----|
| Broken link | Invalid internal link | Check path starts with `/docs/` |
| Missing frontmatter | Page missing title | Add `---` block with required fields |
| MDX error | Invalid JSX in markdown | Escape special characters |

---

## Implementation Order

### Recommended Sequence

1. **Docusaurus config** (title, tagline, theme)
2. **Homepage update** (intro and features)
3. **Sidebar configuration** (auto-generated from folders)
4. **Book intro page** (`docs/intro.md`)
5. **Chapter 1** (foundation, no dependencies)
6. **Chapter 2** (ROS 2, enables later chapters)
7. **Chapter 3** (simulation, needs Ch2)
8. **Chapter 4** (Isaac, needs Ch3)
9. **Chapter 5** (VLA, needs Ch2)
10. **Chapter 6** (capstone, needs all)
11. **Final verification** (full build, link check)

### Time Estimates (Per Chapter)

| Chapter | Pages | Complexity | Estimated Effort |
|---------|-------|------------|------------------|
| Ch1 | 4 | Low | Short |
| Ch2 | 5 | Medium | Medium |
| Ch3 | 4 | Medium | Medium |
| Ch4 | 5 | High | Long |
| Ch5 | 4 | High | Long |
| Ch6 | 4 | High | Long |
