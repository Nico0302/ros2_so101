import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformStamped
import numpy as np
from geometry_msgs.msg import Quaternion
import sys
import time
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import tf_transformations

class PoseTeleoperation(Node):
    """
    Allows teleoperation of a robot's end-effector pose using a 3D tracker.

    Pressing joystick button 7 will set set the reference frame of the tracker to the current position of the end-effector.
    Keeping button 7 pressed and moving the tracker will move the end-effector relative to this position.
    """

    def __init__(self):
        super().__init__("pose_teleoperation")

        self.frame_transform = None

        self.world_frame_id = self.declare_parameter("world_frame_id", "world").get_parameter_value().string_value
        self.tracker_frame_id = self.declare_parameter("tracker_frame_id", "LHR-FF29DD46").get_parameter_value().string_value
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_link").get_parameter_value().string_value
        self.end_effector_frame_id = self.declare_parameter("end_effector_frame_id", "gripper").get_parameter_value().string_value

        self.last_button_cmds = None

        self.tracker_tf_buffer = tf2_ros.Buffer()
        self.tracker_tf_listener = tf2_ros.TransformListener(self.tracker_tf_buffer, self)

        self.end_effector_tf_buffer = tf2_ros.Buffer()
        self.end_effector_tf_listener = tf2_ros.TransformListener(self.end_effector_tf_buffer, self)

        self.joy_topic = self.declare_parameter("joy_topic", "joy").get_parameter_value().string_value

        self.frame_transform_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Joy, self.joy_topic, self.joystick_event_callback, 1
        )

        self.end_effector_pose_topic = self.declare_parameter("end_effector_pose_topic", "target_end_effector_pose").get_parameter_value().string_value

        self.pub = self.create_publisher(PoseStamped, self.end_effector_pose_topic, 3)

        period = 1.0 / self.declare_parameter("publishing_rate", 100).get_parameter_value().double_value
        self.pose_timer = self.create_timer(period, self.publish_target_pose)

        self.frame_timer = self.create_timer(0.1, self.broadcast_frame_transform)
        
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
        
        if msg.buttons[7] == 0 and self.last_button_cmds[7] == 1:
            try:
                self.get_logger().info("Stop teleoperation")
                self.stop_teleoperation()
            except Exception as e:
                self.get_logger().error(f"Failed to stop teleoperation: {e}")
            
        elif msg.buttons[7] == 1 and self.last_button_cmds[1] == 0:
            try:
                self.get_logger().info("Start teleoperation")
                self.start_teleoperation()
            except Exception as e:
                self.get_logger().error(f"Failed to start teleoperation: {e}")
        
        self.last_button_cmds = msg.buttons

    def start_teleoperation(self):
        self.frame_transform = self.get_frame_transform()
        if self.frame_transform is None:
            self.get_logger().error("Could not get frame transform")
            return
        
        self.broadcast_frame_transform()


        

        
    def stop_teleoperation(self):
        self.frame_transform = None

                
    
    def get_frame_transform(self):
        """
        Get a transform between the tracker world frame and the end-effector base frame.

        $$
        w_T_t * e_T_b = w_T_b
        $$
        """

        e_T_b, w_T_t = None, None

        try:
            e_T_b = self.end_effector_tf_buffer.lookup_transform(
                self.end_effector_frame_id,
                self.base_frame_id,
                rclpy.time.Time())
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {self.end_effector_frame_id} to {self.base_frame_id}: {ex}')
            return

        try:
            w_T_t = self.tracker_tf_buffer.lookup_transform(
                self.world_frame_id,
                self.tracker_frame_id,
                rclpy.time.Time())
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {self.world_frame_id} to {self.tracker_frame_id}: {ex}')
            return
        
        if e_T_b is None or w_T_t is None:
            return
        
        return self.multiply_transform_stamped(w_T_t, e_T_b)
        
        
    def multiply_transform_stamped(self, t1: TransformStamped, t2: TransformStamped) -> TransformStamped:
        result = TransformStamped()
        result.header.stamp = t2.header.stamp
        result.header.frame_id = t1.header.frame_id
        result.child_frame_id = t2.child_frame_id

        mat1 = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z]),
            tf_transformations.quaternion_matrix([t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w])
        )
        mat2 = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z]),
            tf_transformations.quaternion_matrix([t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w])
        )
        mat_result = tf_transformations.concatenate_matrices(mat1, mat2)

        trans = tf_transformations.translation_from_matrix(mat_result)
        quat = tf_transformations.quaternion_from_matrix(mat_result)

        result.transform.translation.x, result.transform.translation.y, result.transform.translation.z = trans
        result.transform.rotation.x, result.transform.rotation.y, result.transform.rotation.z, result.transform.rotation.w = quat
        return result
    
    def broadcast_frame_transform(self):
        if self.frame_transform is None:
            return None
        
        self.frame_transform_broadcaster.sendTransform(self.frame_transform)

    def publish_target_pose(self):
        if self.frame_transform is None:
            return None
        
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.tracker_frame_id
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            self.pub.publish(msg)
        except Exception:
            # Swallow 'publish() to closed topic' error.
            # This rarely happens on killing this node.
            pass
        


def main(args=None):
    rclpy.init(args=args)

    node = PoseTeleoperation()
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