import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time


class IncrementalJointController(Node):
    def __init__(self):
        super().__init__("incremental_joint_positions_node")
        
        # Publish to robot joint position controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/incremental_joint_targets",
            10
        )
        
        # ADDED: Subscribe to q_dot commands from linear motion controller
        self.sub_qdot = self.create_subscription(
            Float64MultiArray,
            "/incremental_qdot_cmd",
            self.qdot_cmd_callback,
            10
        )
        
        # Initialize q1,q2,q3,q4 to 0
        self.q_ref = np.zeros(4)
        
        # Store current time for dt calculation
        self.last_time = time.time()
        
        self.get_logger().info("Incremental joint controller ready.")
        
        # Keep updating and publishing every 0.01 seconds => 100Hz
        self.timer = self.create_timer(0.01, self.update_loop)
        
        # Initialize to zero, will be updated by subscription
        self.q_dot_cmd = np.zeros(4)
    
    def qdot_cmd_callback(self, msg):
        """Receives joint velocity commands from linear motion controller"""
        self.q_dot_cmd = np.array(msg.data, dtype=float)
        self.get_logger().debug(f"Received q_dot: {self.q_dot_cmd}")
    
    def set_q_dot(self, q_dot):
        """Externally set new q_dot (for backward compatibility)"""
        self.q_dot_cmd = np.array(q_dot)
    
    def update_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Incremental update: position = position + velocity * time
        self.q_ref = self.q_ref + self.q_dot_cmd * dt
        
        msg = Float64MultiArray()
        msg.data = self.q_ref.tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IncrementalJointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()