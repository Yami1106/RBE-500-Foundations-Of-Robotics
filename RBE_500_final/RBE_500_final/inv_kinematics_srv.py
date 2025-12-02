import rclpy
from rclpy.node import Node

import math

from .utils import inverse_kinematics

from interfaces_pkg.srv import InverseKinematics


class InverseKinematicsServer(Node):
    def __init__(self):
        super().__init__("inv_kinematics_srv")
        self.srv = self.create_service(
            InverseKinematics,
            "inverse_kinematics",
            self.inv_kinematics_cb,
        )
        self.get_logger().info(
            "Inverse Kinematics service 'inverse_kinematics' is ready."
        )

    def inv_kinematics_cb(self, request, response):
        pose = request.pose
        self.get_logger().info(f"Received pose: {pose}")

        q1, q2, q3, q4 = inverse_kinematics(pose)

        # Convert to degrees
        # q1 = math.degrees(q1_rad)
        # q2 = math.degrees(q2_rad)
        # q3 = math.degrees(q3_rad)
        # q4 = math.degrees(q4_rad)

        self.get_logger().info(
            f"IK solution (deg): q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f}, q4={q4:.3f}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    server_node = InverseKinematicsServer()
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
