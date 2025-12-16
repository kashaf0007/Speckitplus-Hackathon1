# Chapter 6 Contract: Capstone Project – The Autonomous Humanoid

**Chapter**: 6
**Directory**: `my-book/docs/chapter-6-capstone/`
**Prerequisites**: Chapters 1-5 (all previous chapters)

---

## Deliverables

### Files to Create

| File | sidebar_position | Purpose |
|------|-----------------|---------|
| `_category_.json` | - | Category metadata |
| `index.md` | 1 | Chapter overview |
| `system-architecture.md` | 2 | Full system design |
| `implementation.md` | 3 | Step-by-step build |
| `testing-validation.md` | 4 | Testing the system |

---

## Content Requirements

### index.md - Chapter Overview

**Must Include**:
- Capstone project description
- What the final system does (voice → plan → navigate → detect → manipulate)
- Prerequisites: All previous chapters completed
- Hardware/software requirements summary
- Expected outcome

### system-architecture.md

**Must Include**:
- End-to-end system architecture
- Component overview from each chapter
- Data flow from voice input to robot action
- Node communication graph
- State machine for task execution
- Error handling strategy

**Code Examples**:
1. High-level launch file structure
2. Node interconnection configuration
3. State machine definition (SMACH or similar)

**Diagrams**:
- Mermaid: Complete system architecture (MUST include)
- Mermaid: Data flow diagram
- Mermaid: State machine diagram
- ASCII: Node communication graph

### implementation.md

**Must Include**:
- Step-by-step implementation guide
- Integration of Whisper node
- Integration of LLM planner node
- Integration of Nav2 navigation
- Integration of perception (object detection)
- Integration of manipulation action
- Testing each component individually

**Code Examples**:
1. Main launch file (complete)
2. Configuration files (params YAML)
3. Custom message definitions (if any)
4. Integration test script
5. Demo command examples

**Diagrams**:
- Mermaid: Implementation sequence

### testing-validation.md

**Must Include**:
- Testing methodology
- Unit testing individual nodes
- Integration testing the pipeline
- End-to-end demo scenarios
- Performance benchmarks
- Common failure modes and debugging
- Success criteria validation

**Code Examples**:
1. pytest test cases for nodes
2. Integration test launch file
3. Demo script with test commands
4. Logging and debugging configuration

**Demo Scenarios**:
1. "Go to the kitchen" - navigation only
2. "Find the red ball" - navigation + detection
3. "Pick up the cup from the table" - full pipeline

**Diagrams**:
- Table: Test case matrix
- Mermaid: Test coverage diagram

---

## Integration Requirements

### Components from Previous Chapters

| Component | Source Chapter | Integration Point |
|-----------|---------------|-------------------|
| ROS 2 nodes | Chapter 2 | All communication |
| URDF humanoid | Chapter 2 | Robot model |
| Gazebo/Isaac simulation | Chapter 3/4 | Simulation environment |
| Sensor simulation | Chapter 3 | Perception input |
| Nav2 navigation | Chapter 4 | Movement actions |
| Whisper ASR | Chapter 5 | Voice input |
| LLM planner | Chapter 5 | Task decomposition |
| Action execution | Chapter 5 | Robot actions |

### Data Flow

```
Voice Input → Whisper → LLM Planner → Task Sequence
                                           ↓
                              ┌────────────┴────────────┐
                              ↓            ↓            ↓
                         Navigate      Detect       Manipulate
                              ↓            ↓            ↓
                         Nav2 Goal   Perception   Action Server
                              ↓            ↓            ↓
                              └────────────┴────────────┘
                                           ↓
                                    Robot Execution
                                           ↓
                                    Feedback to User
```

---

## Acceptance Criteria

- [ ] All 4 content files created with correct frontmatter
- [ ] `_category_.json` with label "6. Capstone" and position 6
- [ ] Complete system architecture diagram in Mermaid
- [ ] Integration of ALL previous chapter components
- [ ] At least 8 code examples total across chapter
- [ ] Working main launch file provided
- [ ] At least 3 demo scenarios documented
- [ ] Testing strategy with specific test cases
- [ ] Performance benchmarks defined
- [ ] At least 5 diagrams across chapter
- [ ] Build succeeds with no broken links
- [ ] Clear prerequisites linking to previous chapters
- [ ] Success criteria from spec SC-010 met
