import rclpy
from rclpy.node import Node
from .utils import (
    get_wrist_position,
    calculate_q1,
    calculate_q2,
    calculate_q3,
    calculate_q4,
)
from team_project.srv import InverseKinematics


class InverseKinematicsServer(Node):
    def __init__(self):
        super().__init__("inv_kinematics_srv")
        self.srv = self.create_service(
            InverseKinematics, "inverse_kinematics", self.inv_kinematics_cb
        )

    def inv_kinematics_cb(self, request, response):
        pose = request.pose
        self.get_logger().info(f"Received pose: {pose}")

        x_c, y_c, z_c = get_wrist_position(pose)

        q1 = calculate_q1(x_c, y_c)
        q2 = calculate_q2(x_c, y_c, z_c)
        q3 = calculate_q3(x_c, y_c, z_c)
        q4 = calculate_q4(x_c, y_c, z_c, pose.orientation)
        response.joint_values = [q1, q2, q3, q4]
        return response


def main():
    rclpy.init()
    server_node = InverseKinematicsServer()
    rclpy.spin(server_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
