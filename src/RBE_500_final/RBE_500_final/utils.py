import math
import numpy as np
from geometry_msgs.msg import Pose, Quaternion

from . import constants as const


# Forward-kinematics



def make_A_matrix(a: float, theta: float, d: float, alpha: float) -> np.ndarray:
    """
    Create the individual transformation matrix A using DH parameters.

    Inputs:
        a, d in mm
        theta, alpha in DEGREES
    """
    alpha_rad = math.radians(alpha)
    theta_rad = math.radians(theta)

    ca = math.cos(alpha_rad)
    sa = math.sin(alpha_rad)
    ct = math.cos(theta_rad)
    st = math.sin(theta_rad)

    A = np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,     sa,      ca,     d   ],
            [0.0,    0.0,     0.0,   1.0  ],
        ]
    )

    return A

# Inverse Kinematics 


# function to calculate pitch from quaternion angles which is phi
def _tool_pitch_from_quaternion(q: Quaternion) -> float:
    """
    Extract the tool pitch φ from the end-effector orientation.

    φ = atan2(R31, R33)
    """
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    r31 = 2.0 * (qx * qy + qw * qz)
    r33 = 1.0 - 2.0 * (qy**2 + qz**2)

    return math.atan2(r31, r33)  # radians


def inverse_kinematics(pose: Pose) -> tuple[float, float, float, float]:
    """
    IK for 4-DOF OpenManipulator-X using DH table:

      Joint 1: θ1
      Joint 2: θ2_DH = q2_cmd − θ0
      Joint 3: θ3_DH = q3_cmd + θ0
      Joint 4: θ4

    Returns:
      q1_cmd, q2_cmd, q3_cmd, q4_cmd in degrees,
      to follow same convention FK expects
    """

    px = pose.position.x * 1000.0
    py = pose.position.y * 1000.0
    pz = pose.position.z * 1000.0

    theta1 = math.atan2(py, px)

    # combine xy plane to r plane now represent everything as (ri,zi)
    # r3, z3 in the r–z plane
    r3 = math.hypot(px, py)
    z3 = pz - const.d1

    phi = _tool_pitch_from_quaternion(pose.orientation)  # in radians

    r2 = r3 - const.a4 * math.cos(phi)
    z2 = z3 - const.a4 * math.sin(phi)

    num = r2**2 + z2**2 - (const.a2**2 + const.a3**2)
    den = 2.0 * const.a2 * const.a3
    D = num / den
    D = max(-1.0, min(1.0, D))         # to keep in check that floating point does not interfere with the values clamp it between -1 and 1 

    theta3 = -math.acos(D)              # our robot cannot do elbow-up so take elbow-down solution (radians) by default

    k1 = const.a2 + const.a3 * math.cos(theta3)
    k2 = const.a3 * math.sin(theta3)

    denom = r2**2 + z2**2
    cos_t2 = (k1 * r2 + k2 * z2) / denom
    sin_t2 = (k1 * z2 - k2 * r2) / denom

    theta2 = math.atan2(sin_t2, cos_t2)  # radians

    theta4 = phi - (theta2 + theta3)     # radians

    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)
    theta4_deg = math.degrees(theta4)


    q1_cmd = theta1_deg
    q2_cmd = theta2_deg + const.angle_offset  # if not done ik gives the geometric angles which differ by the offset value 
    q3_cmd = theta3_deg - const.angle_offset  # not necessary but makes it simpler to read it in the output
    q4_cmd = theta4_deg

    return q1_cmd, q2_cmd, q3_cmd, q4_cmd
