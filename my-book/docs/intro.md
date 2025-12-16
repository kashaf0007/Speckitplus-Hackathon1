---
sidebar_position: 1
slug: /intro
---

# Physical AI & Humanoid Robotics

Welcome to this comprehensive guide on building embodied AI systems. This book takes a **simulation-first approach** to developing humanoid robots, ensuring you can safely experiment and iterate before deploying to real hardware.

## What You'll Learn

This book covers the complete stack for building intelligent humanoid robots:

1. **Physical AI Foundations** - Understanding embodied intelligence and the simulation-first paradigm
2. **ROS 2 Fundamentals** - Building robust robot software with the industry-standard middleware
3. **Simulation Environments** - Mastering Gazebo and Unity for robot development
4. **NVIDIA Isaac** - Leveraging GPU-accelerated simulation and synthetic data
5. **Vision-Language-Action** - Integrating speech, language models, and computer vision
6. **Capstone Project** - Building an autonomous humanoid that responds to voice commands

## Prerequisites

This book assumes familiarity with:

- **Python 3** - Most code examples use Python
- **Linux/Ubuntu** - ROS 2 runs best on Ubuntu 22.04 (Humble)
- **Basic robotics concepts** - Transforms, sensors, actuators
- **Command line** - Terminal proficiency for ROS 2 tooling

## The Simulation-First Philosophy

Why start with simulation?

- **Safety**: Test dangerous scenarios without risking hardware
- **Speed**: Iterate faster than real-time with headless simulation
- **Scale**: Generate synthetic data for ML training
- **Cost**: No hardware required to start learning
- **Reproducibility**: Reset to known states for debugging

## How to Use This Book

Each chapter builds on the previous:

```mermaid
graph LR
    A[Ch 1: Foundations] --> B[Ch 2: ROS 2]
    B --> C[Ch 3: Simulation]
    C --> D[Ch 4: Isaac]
    D --> E[Ch 5: VLA]
    E --> F[Ch 6: Capstone]
```

**Recommended path**: Work through chapters sequentially, completing the exercises in each before moving on.

**Reference use**: Each chapter is self-contained enough for reference once you've completed the initial read-through.

## Getting Started

Ready to begin? Head to [Chapter 1: Introduction to Physical AI](/docs/chapter-1-introduction) to understand the foundations of embodied AI systems.

:::tip Hardware Not Required
You can complete most of this book using only simulation. We'll clearly mark sections that require physical hardware.
:::
