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

        # Subscribe to joint values (Float64MultiArray)
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            "joint_values",          # make sure this matches your publisher
            self.fwd_kinematics_cb,
            10,
        )

        # Publish tool pose as geometry_msgs/Pose
        self.publisher = self.create_publisher(
            Pose,
            "tool_pose",
            10,
        )

    def fwd_kinematics_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != const.DOF:
            self.get_logger().error(
                f"Expected {const.DOF} joint values, but got {len(msg.data)}"
            )
            return

        # Expecting 4 DOF: q1, q2, q3, q4
        q1, q2, q3, q4 = msg.data

        """
        DH parameter table:
        Link |   a   |  θ  |   d        | α
        -------------------------------------
        1    |   0   | q1  |  l0        | 0
        2    |   0   | q2  | l1 + d2    | 270
        3    |  a3   | q3  |  0         | 0
        4    |  l3   | q4  |  0         | 0
        """

        # NOTE: make_A_matrix internally converts theta, alpha from degrees to radians.
        A1 = make_A_matrix(
            a=0.0,
            theta=q1,
            d=const.LINK_0_LENGTH,
            alpha=0.0,
        )
        A2 = make_A_matrix(
            a=0.0,
            theta=q2,
            d=const.LINK_1_LENGTH + const.LINK_2_LENGTH,
            alpha=const.ALPHA_2,
        )
        A3 = make_A_matrix(
            a=const.LINK_3_OFFSET,
            theta=q3,
            d=0.0,
            alpha=0.0,
        )
        A4 = make_A_matrix(
            a=const.LINK_3_LENGTH,
            theta=q4,
            d=0.0,
            alpha=0.0,
        )

        # Tool offset along robot's x axis
        A_tool = np.eye(4)
        A_tool[:3, 3] = [const.LINK_4_LENGTH, 0.0, 0.0]

        # Full transform from base to tool
        T = A1 @ A2 @ A3 @ A4 @ A_tool
        rotation, position = T[:3, :3], T[:3, 3]

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])

        # Rotation matrix -> quaternion (x, y, z, w)
        qx, qy, qz, qw = R.from_matrix(rotation).as_quat()
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)

        self.publisher.publish(pose)
        self.get_logger().info(
            f"Published tool pose: pos=({pose.position.x:.2f}, "
            f"{pose.position.y:.2f}, {pose.position.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
