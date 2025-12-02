import rclpy
from rclpy.node import Node
import numpy as np

from interfaces_pkg.srv import JointVeltoEEVel, EEVeltoJointVel
from .constants import *    
from .utils import make_A_matrix   


# function to calculate the jacobian, take an array input with the values q1,q2,q3,q4 
def compute_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Fill this with J(q) for OpenManipulator-X.

    q: [q1, q2, q3, q4] in radians.

    Return: 6x4 numpy array (linear + angular velocity Jacobian).
    """
    J = np.zeros((6, 4))  # <-- need to replace with our values
    return J

# create/host 2 services
class VelocityKinematics(Node):
    def __init__(self):
        super().__init__("velocity_kinematics")

        # joint velocity to end effector velocity
        self.srv1 = self.create_service(
            JointVeltoEEVel,
            "joint_vel_to_ee_vel",
            self.joint_vel_to_ee_vel_cb
        )

        # end-effector velocity to joint velocity
        self.srv2 = self.create_service(
            EEVeltoJointVel,
            "ee_vel_to_joint_vel",
            self.ee_vel_to_joint_vel_cb
        )

        self.get_logger().info("Velocity kinematics node is ready.")

    # x_dot = J x q_dot 
    def joint_vel_to_ee_vel_cb(self, request, response):
        q = np.array(request.q, dtype=float)
        q_dot = np.array(request.q_dot, dtype=float)

        J = compute_jacobian(q)
        x_dot = J @ q_dot

        response.x_dot = x_dot.tolist()
        return response

    # q_dot = J+ x x_dot, we need the pseudo inverse because the jacobian is not a square matrix
    def ee_vel_to_joint_vel_cb(self, request, response):
        q = np.array(request.q, dtype=float)
        x_dot = np.array(request.x_dot, dtype=float)

        J = compute_jacobian(q)
        J_pinv = np.linalg.pinv(J)

        q_dot = J_pinv @ x_dot
        response.q_dot = q_dot.tolist()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VelocityKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
