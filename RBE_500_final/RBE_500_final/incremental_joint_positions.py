import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class IncrementalJointController(Node):
    def __init__(self):
        super().__init__("incremental_joint_positions_node")

        # publish to robot joint position controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/incremental_joint_targets",
            10
        )

        # initialize q1,q2,q3,q4 to 0
        self.q_ref = np.zeros(4)

        # store current time we will need it for dt calculation 
        self.last_time = time.time()

        self.get_logger().info("Incremental joint controller ready.")

        # keep updating and publishing every 0.01 seconds => 100Hz
        self.timer = self.create_timer(0.01, self.update_loop)

        # initialize it to zero, it will be updated by set_q_dot method
        self.q_dot_cmd = np.zeros(4)

    # externally set new q_dot
    def set_q_dot(self, q_dot):
        self.q_dot_cmd = np.array(q_dot)

    def update_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # incremental update
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
