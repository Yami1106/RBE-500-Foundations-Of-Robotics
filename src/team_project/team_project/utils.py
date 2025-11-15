import numpy as np
from . import constants as const
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation as R

###############################################################################
############################## Utility Functions ##############################
###############################################################################

############################## Forward Kinematics #############################


def make_A_matrix(a: float, theta: float, d: float, alpha: float) -> np.ndarray:
    """Create the individual transformation matrix A using DH parameters."""
    alpha = np.radians(alpha)
    theta = np.radians(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    A = np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ]
    )

    return A


############################## Inverse Kinematics #############################


def get_wrist_position(pose: Pose) -> tuple[float, float, float]:
    """Calculate wrist center position given end-effector pose."""
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    rot = R.from_quat([qx, qy, qz, qw])
    tooltip_axis_world = rot.apply(np.array([1.0, 0.0, 0.0]))
    x_c, y_c, z_c = (
        np.array([pose.position.x, pose.position.y, pose.position.z])
        - const.LINK_4_LENGTH * tooltip_axis_world
    )

    return x_c, y_c, z_c


def calculate_q1(x: float, y: float) -> float:
    """Calculate joint angle q1 (yaw about base) given position."""
    return np.arctan2(y, x)


def _calculate_z3_r3_phi(x: float, y: float, z: float) -> float:
    """Calculate angle phi ."""
    z3 = z - const.LINK_2_OFFSET
    r3 = np.hypot(x, y)
    phi = np.arctan2(z3, r3)
    return z3, r3, phi


def _calculate_r_s(x: float, y: float, z: float) -> tuple[float, float]:
    """Calculate planar components r and s from position."""
    z3, r3, phi = _calculate_z3_r3_phi(x, y, z)
    r, s = (
        r3 - const.LINK_4_LENGTH * np.cos(phi),
        z3 - const.LINK_4_LENGTH * np.sin(phi),
    )
    return r, s


def _cos_law_D(x: float, y: float, z: float) -> float:
    """Calculate the cosine law D value from position."""
    r, s = _calculate_r_s(x, y, z)
    D = (
        r**2
        + s**2
        - const.LINK_2_LENGTH**2
        - const.LINK_3_OFFSET**2
        - const.LINK_3_LENGTH**2
    ) / (2 * const.LINK_3_LENGTH * np.hypot(const.LINK_2_LENGTH, const.LINK_3_OFFSET))
    assert -1 <= D <= 1, "Position is unreachable."
    return D


def _elbow_joint_theta3(D: float) -> float:
    """Calculate elbow joint angle 03 using cosine law D value."""
    return np.arctan2(-np.sqrt(1 - D**2), D)


def calculate_q2(x: float, y: float, z: float) -> float:
    """Calculate joint angle q2 (shoulder pitch) given position."""
    r, s = _calculate_r_s(x, y, z)
    c = np.hypot(const.LINK_2_LENGTH, const.LINK_3_OFFSET)
    q3 = calculate_q3(x, y, z)
    sin_temp = (
        ((c + const.LINK_3_LENGTH * np.cos(q3)) * r)
        + const.LINK_3_LENGTH * np.sin(q3) * s
    ) / (r**2 + s**2)
    cos_temp = (
        ((c + const.LINK_3_LENGTH * np.cos(q3)) * s)
        + const.LINK_3_LENGTH * np.sin(q3) * r
    ) / (r**2 + s**2)
    return np.arctan2(sin_temp, cos_temp)


def calculate_q3(x: float, y: float, z: float) -> float:
    """Calculate joint angle q3 (elbow pitch) given position."""
    return _elbow_joint_theta3(_cos_law_D(x, y, z))


def calculate_q4(x: float, y: float, z: float, orientation: Quaternion) -> float:
    """Calculate joint angle q4 (wrist pitch)."""
    q2 = calculate_q2(x, y, z)
    q3 = calculate_q3(x, y, z)

    _, _, phi = _calculate_z3_r3_phi(x, y, z)
    return phi - q2 - q3
