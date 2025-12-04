import math
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation as R

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

def fwd_kinematics_zero(theta1, theta2, theta3) -> None:
    #if len(msg.position) != const.DOF:
    #    self.get_logger().error(
    #        f"Expected {const.DOF} joint values, but got {len(msg)}"
    #    )
    #    return

    # Expecting 4 DOF: q1, q2, q3, q4
    q1, q2, q3, q4 = np.degrees(theta1), np.degrees(theta2), np.degrees(theta3), 0.0
    """
    DH parameter table:
    Link |   a   |  θ  |   d        | α
    -------------------------------------
    1    |   0   | q1     |  d1        | -90
    2    |  a2   | q2-th  |  0         | 0
    3    |  a3   | q3+th  |  0         | 0
    4    |  a4   | q4     |  0         | 0
    """

    # NOTE: make_A_matrix internally converts theta, alpha from degrees to radians.
    A1 = make_A_matrix(
        a=const.a1,
        theta=q1,
        d=const.d1,
        alpha=const.alpha1,
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

    temp = np.matmul(A1, A2)
    temp = np.matmul(temp, A3)
    T = np.matmul(temp, A4)
    return T

# Inverse Kinematics 

def quat_angle_difference(x1, y1, z1, w1, x2, y2, z2, w2):
    # inverse of q1
    q1_inv = np.array([w1, -x1, -y1, -z1])  # assumes normalized

    # relative quaternion q_rel = q2 * q1_inv
    w = w2*w1 - x2*(-x1) - y2*(-y1) - z2*(-z1)
    x = w2*(-x1) + x2*w1 + y2*(-z1) - z2*(-y1)
    y = w2*(-y1) - x2*(-z1) + y2*w1 + z2*(-x1)
    z = w2*(-z1) + x2*(-y1) - y2*(-x1) + z2*w1

    # angle difference
    angle = 2 * np.arccos(np.clip(abs(w), -1.0, 1.0))

    # sign from 2nd quarternion direction ("upwards direction check")
    axis = np.array([0, 1, 0])
    sign = 0
    if np.dot(np.array([x, y, z]), axis) <= 0:
        sign = 1
    else:
        sign = -1
    
    return angle * sign

def quaternion_to_rotation(q):
    # Turn quarternion to rotation matrix
    x, y, z, w = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    return R

def inverse_kinematics(pose: Pose) -> tuple[float, float, float, float]:
    """
    IK for 4-DOF OpenManipulator-X using DH table:

      Joint 1: θ1
      Joint 2: θ2_DH = q2_cmd + θ0
      Joint 3: θ3_DH = q3_cmd - θ0
      Joint 4: θ4

    Returns:
      q1_cmd, q2_cmd, q3_cmd, q4_cmd in degrees,
      to follow same convention FK expects
    """

    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z

    theta1 = 0
    theta1 = math.atan2(py, px)

    # Alternative Check
    T_tool_rot = quaternion_to_rotation(pose.orientation)
    vx = T_tool_rot[0][0] * const.a4
    vy = T_tool_rot[1][0] * const.a4
    vz = T_tool_rot[2][0] * const.a4

    # Find the Third Joint Position
    x3 = px - vx
    y3 = py - vy
    z3 = pz - vz

    # combine xy plane to r plane now represent everything as (ri,zi)
    # r3, z3 in the r–z plane
    
    r3 = math.hypot(x3, y3)
    z3 = z3 - const.d1

    # Get intermediate variables to solve for q3 Law of Cosines
    num = r3**2 + z3**2 - (const.a2**2 + const.a3**2)
    den = 2.0 * const.a2 * const.a3
    D = num / den
    theta3 = math.atan2(math.sqrt(1 - D**2), D)

    # Account for Robot Offset
    theta3_true = (theta3-math.radians(const.angle_offset))

    # Solve for theta2
    theta2 = (-math.atan2(z3, r3) - math.atan2(const.a3 * math.sin(theta3), 
        const.a2 + const.a3 * math.cos(theta3)))
    theta2_true = np.radians(90) - np.radians(const.short_angle_offset) + theta2

    # Solve for theta4 through analytical method
    comp = fwd_kinematics_zero(theta1, theta2_true, theta3_true)
    T_comp = comp[:3, :3]
    cx, cy, cz, cw = R.from_matrix(T_comp).as_quat()
    sx, sy, sz, sw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    theta4 = quat_angle_difference(cx, cy, cz, cw, sx, sy, sz, sw)

    return theta1, theta2_true, theta3_true, -theta4
    #return np.degrees(theta1), np.degrees(theta2_true), np.degrees(theta3_true), -np.degrees(theta4)
