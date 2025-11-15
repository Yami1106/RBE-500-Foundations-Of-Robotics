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
        1    |   0   | q1     |  d1        | 90
        2    |  a2   | q2-th  |  0         | 0
        3    |  a3   | q3+th  |  0         | 0
        4    |  a4   | q4     |  0         | 0
        """

        # NOTE: make_A_matrix internally converts theta, alpha from degrees to radians.
        A1 = make_A_matrix(
            a=const.a1,
            theta=q1,
            d=const.d1,
            alpha=const.a1,
        )
        A2 = make_A_matrix(
            a=const.a2,
            theta=(q2 - const.angle_offset),
            d=const.d2,
            alpha=const.alpha2,
        )
        A3 = make_A_matrix(
            a=const.a3,
            theta=(q3 + const.angle_offset),
            d=const.d3,
            alpha=const.alpha3,
        )
        A4 = make_A_matrix(
            a=const.a4,
            theta=q4,
            d=const.d4,
            alpha=const.alpha4,
        )

        # Tool offset along robot's x axis
        #A_tool = np.eye(4)
        #A_tool[:3, 3] = [const.a4, 0.0, 0.0]

        # Full transform from base to tool
        T = A1 @ A2 @ A3 @ A4 #@ A_tool
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
