import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess
import numpy as np
import sys
import time

class Teleoperation(Node):
    def __init__(self):
        super().__init__("teleoperation")

        self.pose_topic = self.declare_parameter("pose_topic", "my_pose").value
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.end_effector = self.declare_parameter("end_effector", "gripper").value
        self.last_button_cmds = None

        self.joy_topic = self.declare_parameter("joy_topic", "").value
        if not self.joy_topic:
            self.get_logger().error("joy_topic is not set")
            sys.exit(1)
        self.sub = self.create_subscription(
            Joy, self.joy_topic, self.joystick_event_callback, 1
        )
        
    def joystick_event_callback(self, msg):
        if len(msg.buttons) == 0:
            return

def main(args=None):
    rclpy.init(args=args)

    node = Teleoperation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print(e)
        sys.exit(1)