import numpy as np


def make_A_matrix(a: float, theta: float, d: float, alpha: float) -> np.ndarray:
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
