import numpy as np
from geometry_msgs.msg import Pose, Quaternion

from . import constants as const


###############################################################################
############################## Utility Functions ##############################
###############################################################################


############################## Forward Kinematics #############################


def make_A_matrix(a: float, theta: float, d: float, alpha: float) -> np.ndarray:
    """Create the individual transformation matrix A using DH parameters.

    Inputs:
        a, theta, d, alpha in *degrees* for theta/alpha (a, d in mm).
    """
    alpha_rad = np.radians(alpha)
    theta_rad = np.radians(theta)

    ca = np.cos(alpha_rad)
    sa = np.sin(alpha_rad)
    ct = np.cos(theta_rad)
    st = np.sin(theta_rad)

    A = np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    return A


############################## Inverse Kinematics #############################


# def get_wrist_position(pose: Pose) -> tuple[float, float, float]:
#     """Calculate wrist center position given end-effector pose."""
#     qx = pose.orientation.x
#     qy = pose.orientation.y
#     qz = pose.orientation.z
#     qw = pose.orientation.w

#     # Third column of rotation matrix from quaternion
#     r13 = 2 * (qx * qz + qw * qy)
#     r23 = 2 * (qy * qz - qw * qx)
#     r33 = 1 - 2 * (qx**2 + qy**2)

#     x_c = pose.position.x - const.LINK_4_LENGTH * r13
#     y_c = pose.position.y - const.LINK_4_LENGTH * r23
#     z_c = pose.position.z - const.LINK_4_LENGTH * r33

#     return x_c, y_c, z_c


# def calculate_q1(x: float, y: float) -> float:
#     """Calculate joint angle q1 (yaw about base) given wrist center position."""
#     return np.arctan2(y, x)


# def _planar_components(x: float, y: float, z: float) -> tuple[float, float]:
#     """Calculate planar components r and s from wrist center position."""
#     r = np.hypot(x, y)
#     s = z - const.LINK_2_OFFSET
#     return r, s


# def _cos_law_D(x: float, y: float, z: float) -> float:
#     """Calculate the cosine law D value from wrist center position."""
#     r, s = _planar_components(x, y, z)
#     D = (
#         r**2
#         + s**2
#         - const.LINK_2_LENGTH**2
#         - const.LINK_3_OFFSET**2
#         - const.LINK_3_LENGTH**2
#     ) / (2 * const.LINK_3_LENGTH * np.hypot(const.LINK_2_LENGTH, const.LINK_3_OFFSET))

#     # If unreachable, this assertion will throw.
#     assert -1.0 <= D <= 1.0, "Position is unreachable."
#     return D


# def _elbow_joint_theta3(D: float) -> float:
#     """Calculate elbow joint angle θ3 using cosine law D value."""
#     return np.arctan2(-np.sqrt(1.0 - D**2), D)


# def _offset_angle_beta() -> float:
#     """Calculate the offset angle beta."""
#     return np.arctan2(const.LINK_3_OFFSET, const.LINK_2_LENGTH)


# def calculate_q2(x: float, y: float, z: float) -> float:
#     """Calculate joint angle q2 (shoulder pitch) given wrist center position."""
#     r, s = _planar_components(x, y, z)
#     D = _cos_law_D(x, y, z)
#     theta3 = _elbow_joint_theta3(D)
#     beta = _offset_angle_beta()

#     return (
#         np.arctan2(s, r)
#         - np.arctan2(
#             const.LINK_3_LENGTH * np.sin(theta3),
#             np.hypot(const.LINK_2_LENGTH, const.LINK_3_OFFSET)
#             + const.LINK_3_LENGTH * np.cos(theta3),
#         )
#         + beta
#     )


# def calculate_q3(x: float, y: float, z: float) -> float:
#     """Calculate joint angle q3 (elbow pitch) given wrist center position."""
#     return _elbow_joint_theta3(_cos_law_D(x, y, z)) + _offset_angle_beta()


# def calculate_q4(x: float, y: float, z: float, orientation: Quaternion) -> float:
#     """Calculate joint angle q4 (wrist pitch)."""
#     q2 = calculate_q2(x, y, z)
#     q3 = calculate_q3(x, y, z)

#     # Rotation matrix elements from quaternion
#     qx = orientation.x
#     qy = orientation.y
#     qz = orientation.z
#     qw = orientation.w

#     # R31 and R33 from quaternion
#     r31 = 2 * (qx * qy + qw * qz)
#     r33 = 1 - 2 * (qy**2 + qz**2)

#     return np.arctan2(r31, r33) - (q2 + q3)
