import rclpy
from rclpy.node import Node
import numpy as np
from interfaces_pkg.srv import JointVeltoEEVel, EEVeltoJointVel
from .constants import *
from .utils import make_A_matrix


def compute_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Compute the geometric Jacobian for OpenManipulator-X.
    Uses forward kinematics to ensure consistency with DH parameters.
    
    q: [q1, q2, q3, q4] in radians.
    Return: 6x4 numpy array (linear + angular velocity Jacobian).
    """
    
    # Extract joint angles (in radians)
    q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
    
    # Convert to degrees for DH convention (make_A_matrix expects degrees)
    q1_deg = np.degrees(q1)
    q2_deg = np.degrees(q2)
    q3_deg = np.degrees(q3)
    q4_deg = np.degrees(q4)
    
    # Build transformation matrices using your DH parameters
    A1 = make_A_matrix(a=a1, theta=q1_deg, d=d1, alpha=alpha1)
    A2 = make_A_matrix(a=a2, theta=(q2_deg - angle_offset), d=d2, alpha=alpha2)
    A3 = make_A_matrix(a=a3, theta=(q3_deg + angle_offset), d=d3, alpha=alpha3)
    A4 = make_A_matrix(a=a4, theta=q4_deg, d=d4, alpha=alpha4)
    
    # Compute cumulative transforms
    T1 = A1
    T2 = T1 @ A2
    T3 = T2 @ A3
    T4 = T3 @ A4  # End-effector transform
    
    # Extract positions (origins of each frame)
    o0 = np.array([0.0, 0.0, 0.0])  # Base origin
    o1 = T1[:3, 3]
    o2 = T2[:3, 3]
    o3 = T3[:3, 3]
    o4 = T4[:3, 3]  # End-effector position
    
    # Extract z-axes (rotation axes) from each frame
    z0 = np.array([0.0, 0.0, 1.0])  # Base z-axis
    z1 = T1[:3, 2]
    z2 = T2[:3, 2]
    z3 = T3[:3, 2]
    
    # Build Jacobian using standard formula for revolute joints:
    # J_v = z_{i-1} × (o_n - o_{i-1})  (linear velocity part)
    # J_ω = z_{i-1}                     (angular velocity part)
    
    J = np.zeros((6, 4))
    
    # Joint 1 (revolute about z0)
    J[:3, 0] = np.cross(z0, o4 - o0)  # Linear velocity contribution
    J[3:, 0] = z0                      # Angular velocity contribution
    
    # Joint 2 (revolute about z1)
    J[:3, 1] = np.cross(z1, o4 - o1)
    J[3:, 1] = z1
    
    # Joint 3 (revolute about z2)
    J[:3, 2] = np.cross(z2, o4 - o2)
    J[3:, 2] = z2
    
    # Joint 4 (revolute about z3)
    J[:3, 3] = np.cross(z3, o4 - o3)
    J[3:, 3] = z3
    
    # CRITICAL FIX: If constants are in mm, convert linear part to m
    # Check if values are in mm range (> 10)
    if np.max(np.abs(J[:3, :])) > 10:
        J[:3, :] = J[:3, :] / 1000.0  # Convert mm to m
    
    return J


class VelocityKinematics(Node):
    def __init__(self):
        super().__init__("velocity_kinematics")
        
        # Service: joint velocity -> end-effector velocity
        self.srv1 = self.create_service(
            JointVeltoEEVel,
            "joint_vel_to_ee_vel",
            self.joint_vel_to_ee_vel_cb
        )
        
        # Service: end-effector velocity -> joint velocity
        self.srv2 = self.create_service(
            EEVeltoJointVel,
            "ee_vel_to_joint_vel",
            self.ee_vel_to_joint_vel_cb
        )
        
        self.get_logger().info("Velocity kinematics node ready.")
        self.get_logger().info("Services: /joint_vel_to_ee_vel, /ee_vel_to_joint_vel")
    
    def joint_vel_to_ee_vel_cb(self, request, response):
        """
        Forward velocity kinematics: x_dot = J(q) @ q_dot
        """
        q = np.array(request.q, dtype=float)
        q_dot = np.array(request.q_dot, dtype=float)
        
        # Compute Jacobian at current configuration
        J = compute_jacobian(q)
        
        # Forward velocity mapping
        x_dot = J @ q_dot
        
        response.x_dot = x_dot.tolist()
        
        self.get_logger().debug(f"Joint->EE: q={q}, q_dot={q_dot} -> x_dot={x_dot}")
        
        return response
    
    def ee_vel_to_joint_vel_cb(self, request, response):
        """
        Inverse velocity kinematics: q_dot = J†(q) @ x_dot
        Uses pseudoinverse since J is 6x4 (not square)
        """
        q = np.array(request.q, dtype=float)
        x_dot = np.array(request.x_dot, dtype=float)
        
        # Compute Jacobian at current configuration
        J = compute_jacobian(q)
        
        # Compute pseudoinverse
        J_pinv = np.linalg.pinv(J)
        
        # Inverse velocity mapping
        q_dot = J_pinv @ x_dot
        
        response.q_dot = q_dot.tolist()
        
        self.get_logger().debug(f"EE->Joint: q={q}, x_dot={x_dot} -> q_dot={q_dot}")
        
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