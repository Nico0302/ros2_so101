import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np
from geometry_msgs.msg import Quaternion
import sys
import time
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class Teleoperation(Node):
    def __init__(self):
        super().__init__("teleoperation")

        self.startup_done = False

        self.pose_topic = self.declare_parameter("pose_topic", "my_pose").value
        if not self.pose_topic:
            self.get_logger().error("pose_topic is not set")
            sys.exit(1)
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.end_effector = self.declare_parameter("end_effector", "gripper").value
        self.gripper_action_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.last_button_cmds = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joy_topic = self.declare_parameter("joy_topic", "joy").value
        if not self.joy_topic:
            self.get_logger().error("joy_topic is not set")
            sys.exit(1)

        self.sub = self.create_subscription(
            Joy, self.joy_topic, self.joystick_event_callback, 1
        )

        self.pub = self.create_publisher(PoseStamped, self.pose_topic, 3)

        period = 1.0 / self.declare_parameter("publishing_rate", 100).value
        self.timer = self.create_timer(period, self.publish)
        
    def joystick_event_callback(self, msg):
        if len(msg.buttons) == 0:
            return
        
        if self.last_button_cmds is None:
            self.last_button_cmds = msg.buttons
            return
        
        # log all buttons that have been pressed
        for i, button in enumerate(msg.buttons):
            if button == 1 and self.last_button_cmds[i] == 0:
                self.get_logger().info(f"Button {i} pressed")
        
        if msg.buttons[0] == 1 and self.last_button_cmds[0] == 0:
            try:
                self.get_logger().info("Open gripper")
                self.send_gripper_position(-1.0)
            except Exception as e:
                self.get_logger().error(f"Failed to open gripper: {e}")
        elif msg.buttons[1] == 1 and self.last_button_cmds[1] == 0:
            try:
                self.get_logger().info("Close gripper")
                self.send_gripper_position(1.0)
            except Exception as e:
                self.get_logger().error(f"Failed to close gripper: {e}")
        
        self.last_button_cmds = msg.buttons
                
    def send_gripper_position(self, position: float, max_effort: float = 5.0):
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return self.gripper_action_client.send_goal_async(goal)
        
    def publish(self):
        if not self.startup_done:
            return
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            # msg.pose.position.x = self.pos[0]
            # msg.pose.position.y = self.pos[1]
            # msg.pose.position.z = self.pos[2]
            # msg.pose.orientation.x = self.rot.x
            # msg.pose.orientation.y = self.rot.y
            # msg.pose.orientation.z = self.rot.z
            # msg.pose.orientation.w = self.rot.w

            self.pub.publish(msg)
        except Exception:
            # Swallow 'publish() to closed topic' error.
            # This rarely happens on killing this node.
            pass

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