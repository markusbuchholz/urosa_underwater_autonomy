#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float64MultiArray
import subprocess
import threading
import sys
import re

class TetherCompensatorNode(Node):
    def __init__(self):
        super().__init__('tether_compensator_node')

        # Subscribe to /mission_description: plain text prompt for the LLM
        self.create_subscription(
            String,
            '/mission_description',
            self.mission_callback,
            10
        )
        self.get_logger().info("Listening for mission descriptions on /mission_description…")

        # Publisher for the AUV position result
        self.wp_pub = self.create_publisher(
            Float32MultiArray,
            '/bluerov2/waypoint',
            10
        )
        # Publisher for the ASV position (repurposed /waypoints topic)
        self.asv_pub = self.create_publisher(
            Float64MultiArray,
            '/waypoints',
            10
        )

    def mission_callback(self, msg: String):
        description = msg.data
        self.get_logger().info(f"Received mission description: {description!r}")
        threading.Thread(
            target=self.process_mission,
            args=(description,),
            daemon=True
        ).start()

    def process_mission(self, description: str):
        prompt = description + "\n"
        self.get_logger().info("Running ros2_tether_mission LLM…")

        try:
            result = subprocess.run(
                ["ollama", "run", "ros2_tether_mission"],
                input=prompt,
                text=True,
                capture_output=True,
                check=True
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"LLM execution error: {e.stderr.strip()}")
            return

        output = result.stdout.strip()
        self.get_logger().info(f"LLM output:\n{output}")

        # Parse ASV position: "position ASV: x, y"
        m_asv = re.search(
            r"position\s+ASV:\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)",
            output,
            re.IGNORECASE
        )
        if not m_asv:
            self.get_logger().error("Could not find 'position ASV:' in LLM output.")
        else:
            asv_x = float(m_asv.group(1))
            asv_y = float(m_asv.group(2))
            self.get_logger().info(f"Parsed ASV position: [{asv_x}, {asv_y}]")
            asv_msg = Float64MultiArray(data=[asv_x, asv_y])
            self.asv_pub.publish(asv_msg)
            self.get_logger().info(f"Published ASV to /waypoints: {asv_msg.data}")

        # Parse AUV position: "position AUV: x, y, z"
        m_auv = re.search(
            r"position\s*AUV:\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)",
            output,
            re.IGNORECASE
        )
        if not m_auv:
            self.get_logger().error("Could not find 'position AUV:' in LLM output.")
        else:
            auv_x = float(m_auv.group(1))
            auv_y = float(m_auv.group(2))
            auv_z = float(m_auv.group(3))
            self.get_logger().info(f"Parsed AUV position: [{auv_x}, {auv_y}, {auv_z}]")
            auv_msg = Float32MultiArray(data=[auv_x, auv_y, auv_z])
            self.wp_pub.publish(auv_msg)
            self.get_logger().info(f"Published AUV to /bluerov2/waypoint: {auv_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TetherCompensatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down on user request")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
