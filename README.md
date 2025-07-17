
<p align="center">
  <img src="media/urosa_2.png"
       alt="Urosa screenshot"
       width="500" />
</p>



<h1 align="center">
  Distributed AI Agents for Cognitive Underwater Robot Autonomy
</h1>

---

This repository contains the official implementation of UROSA (Underwater Robot Self-Organizing Autonomy), a framework for building truly autonomous robots using a distributed network of AI agents in ROS 2. <br>

## Introduction
Traditional robotic systems are powerful in controlled settings but often struggle in complex, unpredictable environments like the underwater world. Their reliance on pre-programmed, rule-based algorithms limits their ability to adapt to novel situations, requiring constant human oversight and reprogramming.

## UROSA Presentation

Note: Low quality of audio!<br>
Check the media folder for the better one.


https://github.com/user-attachments/assets/c2a5c82b-b534-4407-9ad8-89c109c9a1c6




## Architecture Overview

![UROSA Architecture Diagram](media/fig_1_ver8.png) 

UROSA is built on a two-layer architecture designed to separate high-level reasoning from low-level control, all orchestrated within the ROS 2 ecosystem.
* **Cognitive Layer:** This is the "brains" of the operation. It contains the distributed network of specialized AI agents that perform tasks like planning, reasoning, and diagnostics.
* **ROS 2 Layer:** This is the "nervous system" of the robot. It handles all communication between agents, interfacing with the robot's hardware (sensors and actuators), and connecting to the simulator.

The fundamental building block of UROSA is the **Agentic ROS 2 Node**, where an LLM is embedded directly inside a ROS 2 node. This makes each agent a first-class citizen in the robotics ecosystem. The **Brain Agent** acts as a central orchestrator and knowledge manager, while **Specialist Agents** (e.g., for Vision, Motion, Diagnostics) execute specific tasks.
Each **Agentic Node**, fuses the high-level AI Agent with its ```ROS 2 Node Implementation```, encapsulating the ```AI Reasoner```, a ```Safety Parser```, and communication interfaces.

## Our Solution: UROSA
UROSA shifts the paradigm from writing rigid, low-level code to setting high-level mission goals. We replace the traditional, monolithic control program with a "cognitive ecosystem" of specialized AI agents. Each agent is an intelligent, ROS 2-native module responsible for a specific task—like vision, motion planning, or diagnostics.

These agents collaborate, reason about their environment, and make decisions autonomously, allowing the system to handle unforeseen events and achieve complex objectives with minimal human intervention.

## Core Innovations
 - **Flexible, Decoupled Reasoning:** Integrates LLM agents into the ROS 2 framework, enabling multiple agents to collaborate on complex problems.
 - **Lifelong Learning & Adaptation:** Agents adapt their behavior in real-time using live sensor data and learn from past experiences to improve performance over time.
 - **Predictive Diagnostics:** An AI agent learns the vehicle's normal behavior to diagnose complex system health issues, moving beyond static fault detection.
 - **On-the-Fly Code Generation:** The system can autonomously write, test, and deploy new ROS 2 nodes at runtime to handle new challenges.
 - **Integrated AI Safety & Control:** Ensures robust system performance through the combination of real-time adaptation, decentralized cognition, and predictive health monitoring.

<p align="center">
  <img src="media/urosa_shift.png"
       alt="Urosa screenshot"
       width="500" />
</p>

## From Traditional to True Autonomy

### Legacy approach

<p align="center">
  <img src="media/legacy.png"
       alt="Urosa screenshot"
       width="700" />
</p>

### AI Autonomy

<p align="center">
  <img src="media/new_urosa.png"
       alt="Urosa screenshot"
       width=700" />
</p>

## Core Mechanisms

UROSA's capabilities are enabled by a series of novel mechanisms that showcase the power of distributed agentic AI. Here’s a simple breakdown of the key innovations:

### 1. Decentralized Reasoning & Multi-Agent Coordination
Instead of a single "brain," UROSA's agents work together as a team. They can communicate with each other to solve complex problems that a single agent could not, such as having two underwater vehicles autonomously negotiate a collision-free path in a cluttered environment.

### Experiential Learning with a Vector Database (RAG)
UROSA agents have a long-term memory. Using a Vector Database (VDB), agents can store and recall past experiences (both good and bad). When faced with a new challenge, like tracking a pipe that becomes hidden, an agent can query its memory to predict where the pipe should be, allowing it to recover much faster than if it were starting from scratch.

### 2. On-the-Fly Code Generation
When UROSA's Brain Agent identifies a missing capability—like a specific data filter or a new planning algorithm—it can task a specialized `Node Gen` agent to **write, test, and deploy a new ROS 2 node at runtime**. The system literally extends its own software functionality without any human intervention.

### 3. Dynamic System Diagnostics
A dedicated Diagnostic Agent continuously monitors the robot's health data (e.g., thruster power, sensor readings). By learning what "normal" looks like, it can diagnose complex, non-obvious failures (like a sluggish thruster) that would be missed by traditional systems that only check for pre-defined error codes.

### 4. Online Behavioral Tuning (Teacher-Student)
One agent can "teach" another to improve its behavior in real-time. For example, a Teacher agent can provide feedback to a Student vision agent, guiding it to make its textual descriptions more concise or to focus on specific objects in a scene, effectively refining its policy on-the-fly.

### 5. Inherent Safety
Safety is built-in at multiple levels. Every agent's behavior is constrained by a "scaffolding" prompt during its creation, and its final output is checked by a `Safety Parser` before being executed. This multi-layered approach ensures that the agents act in a predictable, verifiable, and safe manner.

---
## Getting Started

This section will guide you through setting up the UROSA environment and running the core framework.

### Prerequisites

- Docker
- NVIDIA Docker support for GPU acceleration (recommended)
- StoneFish underwater simulator

### Installation and Setup

1.  **Clone the Repository**

    First, clone the UROSA repository to your local machine:

    ```bash
    git clone https://github.com/markusbuchholz/urosa_underwater_autonomy
    cd urosa_underwater_autonomy/docker
    ```

2.  **Build the Docker Image**

    The provided script will build the Docker image with all the necessary dependencies, including ROS 2 and the UROSA packages.

    ```bash
    sudo ./build.sh
    ```

3.  **Run the Docker Container**

    This command will start the Docker container and give you an interactive shell within the UROSA environment.

    ```bash
    sudo ./run.sh
    ```

## Run UROSA

UROSA's agents are powered by LLMs. We use Ollama to run these models locally. Follow these steps to set up the required AI agent.

1.  **Install Ollama**

    Ollama is a tool for running LLMs locally. Open a new terminal on your host machine (outside the Docker container) and run the following command to install it:

    ```bash
    curl -fsSL [https://ollama.com/install.sh](https://ollama.com/install.sh) | sh
    ```
    This script downloads and installs Ollama on your system.

2.  **Pull a Base Model**

    Next, you need a base model from which to create our specialized ROS 2 agent. We will use e.g. ```llama3```.

    All models are available [here](https://ollama.com/search).

    ```bash
    ollama pull llama3
    ```
    This command downloads the pre-trained Llama 3 model to your machine.

4.  **Create a Custom Model File**

    To make the LLM act as a specialized ROS 2 agent, we need to give it a specific ```system prompt```. We first create a template from the existing `llama3` model.

    The model file specification can be found [here](https://github.com/ollama/ollama/blob/main/docs/modelfile.md).

    ```bash
    ollama show --modelfile llama3 > ros2_model_file
    ```
    This command extracts the configuration (Modelfile) of the `llama3` model and saves it to a file named `ros2_model_file`.

6.  **Define the Agent's Behavior**

    Open the `ros2_model_file` with a text editor. You will see a `SYSTEM` parameter. This is the core instruction that defines the AI's personality, capabilities, and constraints. **Modify the `SYSTEM` prompt** to define the behavior of your ROS 2 agent. For example:

    ```
    # Modelfile generated by "ollama show"
    # To build a new model, create a new file with this content and update the Modelfile
    # (e.g. FROM, PARAMETER, TEMPLATE, SYSTEM, ADAPTER)

    FROM llama3

    # set the temperature to 1 [higher is more creative, lower is more coherent]
    PARAMETER temperature 1

    # The SYSTEM prompt is the most important part.
    # It sets the instructions for the AI agent.
    SYSTEM """
    You are a helpful ROS 2 expert AI assistant.
    Your role is to assist with tasks related to the Robot Operating System 2 (ROS 2).
    You should be able to understand ROS 2 concepts, generate code snippets, debug issues, and provide clear explanations.
    You must always provide safe and valid ROS 2 code and commands.
    """
    ```

7.  **Create the Custom Agent**

    Now, create the new agent model using your modified `ros2_model_file`.

    ```bash
    ollama create ros2_ai_agent --file ros2_model_file
    ```
    This command bundles your custom system prompt and the base model into a new, specialized model named `ros2_ai_agent`.

8.  **Run Your Custom Agent**

    You can now run your custom agent and interact with it directly from the command line.

    ```bash
    ollama run ros2_ai_agent
    ```
    You are now ready to integrate this running agent with the UROSA framework inside the Docker container.


9. **Integrate the Custom Agent into a ROS 2 Node**

    Your ```custom LLM agent``` can be incorporated into a ```ROS 2 node```, allowing it to become an ```agentic``` component that can receive information and perform actions within the ROS 2 ecosystem. This is achieved by creating a ROS 2 node that communicates with the ```Ollama model```.

    The fundamental architecture involves using ROS 2 subscribers and publishers:

    - A ```subscriber``` listens to a topic for incoming data, which is then used to prompt the LLM.

    - The LLM's output, which can be tailored using the ```SYSTEM prompt``` in your Modelfile, is then ```parsed```.

    - A ```publisher``` sends this parsed information as a message on another topic for other nodes to use.

    This cycle typically runs whenever new information is received on the subscribed topics and the LLM is ready to process a new request.

    Example: A Mission-to-Pose Agent
    
    Let's consider an example where a ROS 2 node listens for a mission description on a topic, asks the LLM to extract a target position, and then publishes that position as a PoseStamped message.

    You can initiate the process by sending a mission string to the /rov_mission topic:

    ```bash
    ros2 topic pub /rov_mission std_msgs/msg/String "{data: 'Go to position x = 0, y = -2 , and z = -5'}" --once
     ```
    Below is the Python script for the ROS 2 node (ros2_llm_node.py) that facilitates this interaction:

    NOTE: The PID implemention is not provided.

    To run following node, save the code as ```ros2_llm_node.py``` and execute it:

    ```bash
    source /opt/ros/humble/setup.bash

    python3 ros2_llm_node.py
    ```

    ```python

        #!/usr/bin/env python3
        import subprocess
        import re
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped

        class Ros2LLMChatPublisher(Node):
            def __init__(self):
                super().__init__('ros2_llm_chat_publisher')

                # 1. Publisher for the output PoseStamped message
                self.pose_pub = self.create_publisher(PoseStamped, '/pid/request', 10)

                # 2. Subscriber for the incoming mission strings
                self.create_subscription(
                    String,
                    '/rov_mission',
                    self.mission_callback,
                    10
                )

                self.get_logger().info("ROS 2 LLM node started, listening on /rov_mission")

            def mission_callback(self, msg: String):
                """
                This function is triggered whenever a new message is received.
                """
                mission_text = msg.data.strip()
                self.get_logger().info(f"Received mission: \"{mission_text}\"")

                # 3. Call the Ollama LLM with the mission text as a prompt
                command = ["ollama", "run", "ros2_ai_agent"]
                try:
                    result = subprocess.run(
                        command,
                        input=mission_text,
                        text=True,
                        capture_output=True,
                        check=True
                    )
                    llm_output = result.stdout.strip()
                    self.get_logger().info(f"LLM output: {llm_output}")

                    # 4. Parse the LLM's output to find the coordinates
                    # The LLM is prompted to return data in a specific format.
                    m = re.search(
                        r"position:\s*\{[^}]*x:\s*([-\d\.]+),\s*y:\s*([-\d\.]+),\s*z:\s*([-\d\.]+)",
                        llm_output
                    )
                    if not m:
                        self.get_logger().warn("Could not parse position from LLM output")
                        return

                    x_str, y_str, z_str = m.groups()
                    x, y, z = float(x_str), float(y_str), float(z_str)

                    # 5. Publish the extracted coordinates as a PoseStamped message
                    self.publish_pose(x, y, z)

                except subprocess.CalledProcessError as e:
                    self.get_logger().error(f"LLM call failed: {e.stderr.strip()}")
                except Exception as e:
                    self.get_logger().error(f"Unexpected error: {e}")

            def publish_pose(self, x: float, y: float, z: float):
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "base_link"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.w = 1.0 # Neutral orientation

                self.pose_pub.publish(pose)
                self.get_logger().info(f"Published PoseStamped -> x={x}, y={y}, z={z}")

        def main(args=None):
            rclpy.init(args=args)
            node = Ros2LLMChatPublisher()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info("Shutdown requested, exiting.")
            finally:
                node.destroy_node()
                rclpy.shutdown()

        if __name__ == '__main__':
            main()

    ```
   
---

## Citation
```bibtex
@inproceedings{buchholzUROSA2025,
  author    = {Buchholz, Markus and Carlucho, Ignacio and Grimaldi, Michele and Petillot, Yvan R.},
  title     = {Distributed AI Agents for Cognitive Underwater Robot Autonomy},
  booktitle = {Proceedings of the ... Conference},
  year      = {2025}
}
