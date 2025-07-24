#!/usr/bin/env python3
#ros2 topic pub /map_mission std_msgs/msg/String "{data: 'Compute the error'}" --once

import os
import subprocess
import threading
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Directories and file paths
LLM_IMAGES_DIR    = os.path.join(os.getcwd(), "llm_images")
MAP_IMAGES_DIR    = os.path.join(os.getcwd(), "map_images") 
ERROR_FILE_PATH   = os.path.join(LLM_IMAGES_DIR, "errors.txt")

class MapPlannerNode(Node):
    def __init__(self):
        super().__init__('object_planner')

        # Subscribe to /map_mission for textual mission descriptions
        self.create_subscription(
            String,
            '/map_mission',
            self.map_mission_callback,
            10
        )
        self.get_logger().info("Listening for map missions on /map_mission…")

    def map_mission_callback(self, msg: String):
        mission_text = msg.data.strip()
        self.get_logger().info(f"Received mission text: {mission_text!r}")
        # Offload LLM call to a thread so we don't block the ROS executor
        threading.Thread(
            target=self.process_request,
            args=(mission_text,),
            daemon=True
        ).start()

    def process_request(self, mission_text: str):
        # --- verify directories and files exist ---
        if not os.path.isdir(MAP_IMAGES_DIR):
            self.get_logger().error(f"Map images directory not found: {MAP_IMAGES_DIR}")
            return
        if not os.path.isdir(LLM_IMAGES_DIR):
            self.get_logger().error(f"LLM images directory not found: {LLM_IMAGES_DIR}")
            return
        if not os.path.isfile(ERROR_FILE_PATH):
            self.get_logger().error(f"Errors file not found: {ERROR_FILE_PATH}")
            return

        # --- gather LLM reference images + errors ---
        llm_image_files = sorted([
            os.path.join(LLM_IMAGES_DIR, f)
            for f in os.listdir(LLM_IMAGES_DIR)
            if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))
        ])
        if not llm_image_files:
            self.get_logger().error(f"No image files found in: {LLM_IMAGES_DIR}")
            return

        with open(ERROR_FILE_PATH, 'r') as ef:
            errors_content = ef.read().strip()

        # --- gather all map images ---
        map_image_files = sorted([
            os.path.join(MAP_IMAGES_DIR, f)
            for f in os.listdir(MAP_IMAGES_DIR)
            if f.lower().endswith('.png')
        ])
        if not map_image_files:
            self.get_logger().error(f"No PNG map files found in: {MAP_IMAGES_DIR}")
            return

        # --- process each map / ask the LLM separately ---
        for map_path in map_image_files:
            prompt = (
                f"Mission: {mission_text}\n\n"
                "Reference images:\n" +
                "\n".join(llm_image_files) +
                "\n\nCorresponding errors:\n" +
                errors_content +
                "\n\nCompute the error for this map image:\n" +
                map_path +
                "\n"
            )
            self.get_logger().info(f"Prompt to LLM for {os.path.basename(map_path)}:\n{prompt}")

            try:
                result = subprocess.run(
                    ["ollama", "run", "ros2_vision_vdb"],
                    input=prompt,
                    text=True,
                    capture_output=True,
                    check=True
                )
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"LLM execution error on {map_path}: {e.stderr.strip()}")
                continue

            output = result.stdout.strip()

            self.get_logger().info(f"Result for {os.path.basename(map_path)}:\n{output}")
            print("\n===== LLM OUTPUT =====")
            print(output)
            print("======================\n")


def main(args=None):
    rclpy.init(args=args)
    node = MapPlannerNode()
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

