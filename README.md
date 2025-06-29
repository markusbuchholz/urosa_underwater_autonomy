# UROSA: Distributed AI Agents for Cognitive Underwater Robot Autonomy
---

This repository contains the official implementation of UROSA (Underwater Robot Self-Organizing Autonomy), a framework for building truly autonomous robots using a distributed network of AI agents in ROS 2. <br>

## Introduction
Traditional robotic systems are powerful in controlled settings but often struggle in complex, unpredictable environments like the underwater world. Their reliance on pre-programmed, rule-based algorithms limits their ability to adapt to novel situations, requiring constant human oversight and reprogramming.

## Our Solution: UROSA
UROSA shifts the paradigm from writing rigid, low-level code to setting high-level mission goals. We replace the traditional, monolithic control program with a "cognitive ecosystem" of specialized AI agents. Each agent is an intelligent, ROS 2-native module responsible for a specific task—like vision, motion planning, or diagnostics.

These agents collaborate, reason about their environment, and make decisions autonomously, allowing the system to handle unforeseen events and achieve complex objectives with minimal human intervention.

Key Features
- 🤖 Agentic AI in ROS 2: Seamlessly integrate powerful, pre-trained LLM agents directly into your ROS 2 computation graph.
- 🧠 Decentralized Cognition: Create robust, fault-tolerant systems where multiple agents work together to solve problems.
- ⚡ Dynamic Adaptation: Agents can reason about their environment and adapt their behavior in real-time based on live sensor data or changing mission needs.
- ✍️ On-the-Fly Code Generation: The system can autonomously write, test, and deploy new ROS 2 nodes at runtime to extend its own functionality when it encounters a new challenge.
- 📈 Advanced Diagnostics: Move beyond static fault trees with an AI agent that can diagnose complex system health issues by learning the vehicle's normal behavior.
- 📚 Experiential Learning: Utilize a Vector Database to enable agents to learn from past experiences, improving performance and robustness over time.

## Getting Started
This repository provides the necessary software and instructions to run the core UROSA framework and replicate the experiments presented in our paper.


## Architecture Overview

UROSA is built on a two-layer architecture designed to separate high-level reasoning from low-level control, all orchestrated within the ROS 2 ecosystem.

![UROSA Architecture Diagram](media/fig_1_ver4.png) 

* **Cognitive Layer:** This is the "brains" of the operation. It contains the distributed network of specialized AI agents that perform tasks like planning, reasoning, and diagnostics.
* **ROS 2 Layer:** This is the "nervous system" of the robot. It handles all communication between agents, interfacing with the robot's hardware (sensors and actuators), and connecting to the simulator.

The fundamental building block of UROSA is the **Agentic ROS 2 Node**, where an LLM is embedded directly inside a ROS 2 node. This makes each agent a first-class citizen in the robotics ecosystem. The **Brain Agent** acts as a central orchestrator and knowledge manager, while **Specialist Agents** (e.g., for Vision, Motion, Diagnostics) execute specific tasks.

## Core Mechanisms

UROSA's capabilities are enabled by a series of novel mechanisms that showcase the power of distributed agentic AI. Here’s a simple breakdown of the key innovations:

### Decentralized Reasoning & Multi-Agent Coordination
Instead of a single "brain," UROSA's agents work together as a team. They can communicate with each other to solve complex problems that a single agent could not, such as having two underwater vehicles autonomously negotiate a collision-free path in a cluttered environment.

### Experiential Learning with a Vector Database (RAG)
UROSA agents have a long-term memory. Using a Vector Database (VDB), agents can store and recall past experiences (both good and bad). When faced with a new challenge, like tracking a pipe that becomes hidden, an agent can query its memory to predict where the pipe should be, allowing it to recover much faster than if it were starting from scratch.

### On-the-Fly Code Generation
When UROSA's Brain Agent identifies a missing capability—like a specific data filter or a new planning algorithm—it can task a specialized `Node Gen` agent to **write, test, and deploy a new ROS 2 node at runtime**. The system literally extends its own software functionality without any human intervention.

### Dynamic System Diagnostics
A dedicated Diagnostic Agent continuously monitors the robot's health data (e.g., thruster power, sensor readings). By learning what "normal" looks like, it can diagnose complex, non-obvious failures (like a sluggish thruster) that would be missed by traditional systems that only check for pre-defined error codes.

### Online Behavioral Tuning (Teacher-Student)
One agent can "teach" another to improve its behavior in real-time. For example, a Teacher agent can provide feedback to a Student vision agent, guiding it to make its textual descriptions more concise or to focus on specific objects in a scene, effectively refining its policy on-the-fly.

### Inherent Safety
Safety is built-in at multiple levels. Every agent's behavior is constrained by a "scaffolding" prompt during its creation, and its final output is checked by a `Safety Parser` before being executed. This multi-layered approach ensures that the agents act in a predictable, verifiable, and safe manner.


```bash
@inproceedings{buchholzUROSA2025,
  author    = {Buchholz, Markus and Carlucho, Ignacio and Grimaldi, Michele and Petillot, Yvan R.},
  title     = {Distributed AI Agents for Cognitive Underwater Robot Autonomy},
  booktitle = {Proceedings of the ... Conference},
  year      = {2025}
}
```
