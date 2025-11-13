import rclpy
from rclpy.node import Node

from .utils import (
    get_wrist_position,
    calculate_q1,
    calculate_q2,
    calculate_q3,
    calculate_q4,
)

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

        x_c, y_c, z_c = get_wrist_position(pose)

        q1 = calculate_q1(x_c, y_c)
        q2 = calculate_q2(x_c, y_c, z_c)
        q3 = calculate_q3(x_c, y_c, z_c)
        q4 = calculate_q4(x_c, y_c, z_c, pose.orientation)

        # Store as list[float]
        response.joint_positions = [float(q1), float(q2), float(q3), float(q4)]
        self.get_logger().info(
            f"IK solution: q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f}, q4={q4:.3f}"
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
