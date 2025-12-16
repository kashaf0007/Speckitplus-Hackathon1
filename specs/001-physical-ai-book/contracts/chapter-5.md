# Chapter 5 Contract: Vision-Language-Action (VLA) Systems

**Chapter**: 5
**Directory**: `my-book/docs/chapter-5-vla/`
**Prerequisites**: Chapter 2 (ROS 2 fundamentals)

---

## Deliverables

### Files to Create

| File | sidebar_position | Purpose |
|------|-----------------|---------|
| `_category_.json` | - | Category metadata |
| `index.md` | 1 | Chapter overview |
| `voice-to-action.md` | 2 | Speech processing |
| `llm-cognitive-planning.md` | 3 | LLM task planning |
| `nlp-to-ros2.md` | 4 | Action execution |

---

## Content Requirements

### index.md - Chapter Overview

**Must Include**:
- Learning objectives (Whisper, LLM planning, action grounding)
- What is Vision-Language-Action (VLA)
- The convergence of LLMs and robotics
- Prerequisites: Chapter 2, OpenAI API key (or local alternative)
- Privacy and safety considerations

### voice-to-action.md

**Must Include**:
- Speech-to-text overview
- OpenAI Whisper API usage
- Local Whisper model deployment
- Audio capture in Python
- Integrating with ROS 2 (audio topic)
- Handling noise and ambient sounds

**Code Examples**:
1. Whisper API transcription (Python)
2. Local Whisper model usage
3. PyAudio capture script
4. ROS 2 audio subscriber node
5. Streaming transcription example

**Diagrams**:
- Mermaid: Voice-to-text pipeline

**Alternatives**: Vosk for offline-first, Google Speech API

### llm-cognitive-planning.md

**Must Include**:
- LLM as cognitive layer for robots
- Prompt engineering for task decomposition
- Structured output (JSON) for actions
- Function calling pattern
- Safety constraints and validation
- Handling ambiguous commands

**Code Examples**:
1. System prompt for robot planner
2. Function calling schema definition
3. Task decomposition example
4. Validation and safety checks
5. Error handling for invalid commands

**Diagrams**:
- Mermaid: LLM planning architecture
- Mermaid: Function calling flow

**Alternatives**: Llama 3, Mistral for local deployment

### nlp-to-ros2.md

**Must Include**:
- Mapping LLM output to ROS 2 actions
- Action client implementation
- Feedback loop to user
- Handling action failures
- Multi-step task execution
- Cancellation and preemption

**Code Examples**:
1. Action mapping dictionary
2. ROS 2 action client for navigation
3. Feedback callback implementation
4. Multi-action sequencer node
5. Complete voice-to-navigation pipeline

**Diagrams**:
- Mermaid: NLP to ROS 2 action flow
- Mermaid: Feedback loop architecture

---

## Acceptance Criteria

- [ ] All 4 content files created with correct frontmatter
- [ ] `_category_.json` with label "5. VLA Systems" and position 5
- [ ] Whisper API and local model both documented
- [ ] At least 12 code examples total across chapter
- [ ] Function calling pattern fully explained
- [ ] Safety validation for LLM outputs demonstrated
- [ ] Complete voice-to-action pipeline provided
- [ ] Local model alternatives documented for each API
- [ ] At least 4 Mermaid diagrams
- [ ] Build succeeds with no broken links
- [ ] Warning about LLM hallucinations and safety
