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
