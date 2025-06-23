# UROSA: Distributed AI Agents for Cognitive Underwater Robot Autonomy

This repository contains the official implementation of UROSA (Underwater Robot Self-Organizing Autonomy), a groundbreaking framework for building truly autonomous robots using a distributed network of Large Language Model (LLM) agents in ROS 2. <br>
---
## The Challenge
Traditional robotic systems are powerful in controlled settings but often struggle in complex, unpredictable environments like the underwater world. Their reliance on pre-programmed, rule-based algorithms limits their ability to adapt to novel situations, requiring constant human oversight and reprogramming.

## Our Solution: UROSA
UROSA shifts the paradigm from writing rigid, low-level code to setting high-level mission goals. We replace the traditional, monolithic control program with a "cognitive ecosystem" of specialized AI agents. Each agent is an intelligent, ROS 2-native module responsible for a specific task—like vision, motion planning, or diagnostics.

These agents collaborate, reason about their environment, and make decisions autonomously, allowing the system to handle unforeseen events and achieve complex objectives with minimal human intervention.

Key Features
🤖 Agentic AI in ROS 2: Seamlessly integrate powerful, pre-trained LLM agents directly into your ROS 2 computation graph.
🧠 Decentralized Cognition: Create robust, fault-tolerant systems where multiple agents work together to solve problems.
⚡ Dynamic Adaptation: Agents can reason about their environment and adapt their behavior in real-time based on live sensor data or changing mission needs.
✍️ On-the-Fly Code Generation: The system can autonomously write, test, and deploy new ROS 2 nodes at runtime to extend its own functionality when it encounters a new challenge.
📈 Advanced Diagnostics: Move beyond static fault trees with an AI agent that can diagnose complex system health issues by learning the vehicle's normal behavior.
📚 Experiential Learning: Utilize a Vector Database to enable agents to learn from past experiences, improving performance and robustness over time.

## Getting Started
This repository provides the necessary software and instructions to run the core UROSA framework and replicate the experiments presented in our paper.


```bash
@inproceedings{buchholzUROSA2025,
  author    = {Buchholz, Markus and Carlucho, Ignacio and Grimaldi, Michele and Petillot, Yvan R.},
  title     = {Distributed AI Agents for Cognitive Underwater Robot Autonomy},
  booktitle = {Proceedings of the ... Conference},
  year      = {2025}
}
```
