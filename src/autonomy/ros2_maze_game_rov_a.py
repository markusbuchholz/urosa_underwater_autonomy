#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading

class DualAgentLLM(Node):
    def __init__(self):
        super().__init__('agent_A')
        # Subscribers for individual robot topics
        self.create_subscription(String, '/robot_a', self.robot_a_callback, 10)
        self.create_subscription(String, '/robot_b', self.robot_b_callback, 10)
        #self.get_logger().info("DualAgentLLM node started. Waiting for /robot_a and /robot_b messages.")

        # Publishers for the parsed LLM actions (unchanged topics)
        self.pub_a = self.create_publisher(String, '/actions_from_llm_agent_rov_a', 10)
        self.pub_b = self.create_publisher(String, '/actions_from_llm_agent_rov_b', 10)

        # Storage for incoming data
        self.robot_a_data = None
        self.robot_b_data = None

        # Optional counter to track LLM calls.
        self.counter = 0

    def robot_a_callback(self, msg: String):
        self.get_logger().info(f"Received message on /robot_a: {msg.data}")
        self.robot_a_data = msg.data.strip()
        self.check_and_process()

    def robot_b_callback(self, msg: String):
        #self.get_logger().info(f"Received message on /robot_b: {msg.data}")
        self.robot_b_data = msg.data.strip()
        self.check_and_process()

    def check_and_process(self):
        # Only process if both topics have new data.
        if self.robot_a_data is not None and self.robot_b_data is not None:
            # Process in a separate thread so we don't block callbacks.
            thread = threading.Thread(target=self.process_message, daemon=True)
            thread.start()

    def process_message(self):

        # Build the prompt with static labels and dynamic data.
        prompt = f"path_A: {self.robot_a_data}, path_B: {self.robot_b_data}\n"
        self.get_logger().info(f"Built prompt: {prompt.strip()}")
        try:
            result = subprocess.run(
                ["ollama", "run", "ros2_robot_game_rov_a"],
                input=prompt,
                text=True,
                capture_output=True,
                check=True
            )
            llm_output = result.stdout.strip()
            self.counter += 1
            self.get_logger().info(f"LLM Agent A (call #{self.counter}): {llm_output}")
            self.parse_llm_output(llm_output)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"LLM execution error: {e.stderr.strip()}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")
        finally:
            # Clear the stored data so the next pair of messages triggers a new call.
            self.robot_a_data = None
            self.robot_b_data = None

    def parse_llm_output(self, llm_output: str):
        """
        Parses the LLM output and publishes the actions for robot A and robot B.
        Expected output format from the LLM should include lines like:
            robot_a: <action string>
            robot_b: <action string>
        """
        lines = llm_output.splitlines()
        for line in lines:
            if "robot_a:" in line:
                parts = line.split("robot_a:")
                if len(parts) > 1:
                    action_str = parts[1].strip()
                    msg = String()
                    msg.data = action_str
                    self.pub_a.publish(msg)
                    self.get_logger().info(f"Published to /actions_from_llm_agent_rov_a: {action_str}")
            elif "robot_b:" in line:
                parts = line.split("robot_b:")
                if len(parts) > 1:
                    action_str = parts[1].strip()
                    msg = String()
                    msg.data = action_str


def main(args=None):
    rclpy.init(args=args)
    node = DualAgentLLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
