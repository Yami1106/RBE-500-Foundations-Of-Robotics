import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import numpy as np
from .utils import make_A_matrix
from . import constants as const


class ForwardKinematics(Node):
    def __init__(self):
        super().__init__("fwd_kinematics")
        self.subscriber = self.create_subscription(
            Float64MultiArray, "joint_values", self.fwd_kinematics_cb, 10
        )
        self.publisher = self.create_publisher(Pose, "tool_pose", 10)

    def fwd_kinematics_cb(self, msg):
        if len(msg.data) != const.DOF:
            self.get_logger().error(
                f"Expected {const.DOF} joint values, but got {len(msg.data)}"
            )
            return

        q1, q2, q3, q4 = msg.data

        """
        DH parameter table:
        Link | a   | θ   |   d   | α
        ---------------------------
        1    | 0   | q1  |  l0   | 0
        2    | 0   | q2  | l1+d2 | 270
        3    | a3  | q3  |   0   | 0
        4    | l3  | q4  |   0   | 0

        """
        A1 = make_A_matrix(
            a=0,
            theta=q1,
            d=const.LINK_0_LENGTH,
            alpha=0,
        )
        A2 = make_A_matrix(
            a=0,
            theta=q2,
            d=const.LINK_1_LENGTH + const.LINK_2_LENGTH,
            alpha=const.ALPHA_2,
        )
        A3 = make_A_matrix(
            a=const.LINK_3_OFFSET,
            theta=q3,
            d=0,
            alpha=0,
        )
        A4 = make_A_matrix(
            a=const.LINK_3_LENGTH,
            theta=q4,
            d=0,
            alpha=0,
        )
        # tool offset along robot's x axis
        A_tool = np.eye(4)
        A_tool[:3, 3] = [const.LINK_4_LENGTH, 0.0, 0.0]

        T = A1 @ A2 @ A3 @ A4 @ A_tool
        rotation, position = T[:3, :3], T[:3, 3]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = R.from_matrix(rotation).as_quat()
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    fwd_kinematics_node = ForwardKinematics()
    rclpy.spin(fwd_kinematics_node)
    fwd_kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
