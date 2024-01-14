# Helper to do multi-dimensional trajectory planning.
from typing import Union

import numpy as np


def get_trajectory_fn(a_0: Union[float, np.ndarray], a_1: Union[float, np.ndarray], a_2: Union[float, np.ndarray],
                      a_3: Union[float, np.ndarray]):
    """
    Return cubic trajectory function that will return position, velocity, and acceleration
    for the given time.

    x(t) = a_0 + a_1*t + a_2*t^2 + a_3*t^3

    Each a_i should be Nx1, where N is the dim of the trajectory.
    """

    def trajectory(t: float):
        x = a_0 + (a_1 * t) + (a_2 * (t ** 2)) + (a_3 * (t ** 3))
        v = a_1 + (2 * a_2 * t) + (3 * a_3 * (t ** 2))
        a = (2 * a_2) + (6 * a_3 * t)
        return x, v, a

    return trajectory


def get_linear_trajectory(x_0: Union[float, np.ndarray], v_0: Union[float, np.ndarray]):
    a_0 = x_0
    a_1 = v_0
    a_2 = a_3 = np.zeros_like(x_0)
    return get_trajectory_fn(a_0, a_1, a_2, a_3)


def get_quadratic_trajectory(x_0: Union[float, np.ndarray], v_0: Union[float, np.ndarray],
                             x_t: Union[float, np.ndarray], t: float):
    a_0 = x_0
    a_1 = v_0
    a_2 = (x_t - x_0 - (v_0 * t)) / (t ** 2)
    a_3 = np.zeros_like(x_0)
    return get_trajectory_fn(a_0, a_1, a_2, a_3)


def get_cubic_trajectory(x_0: Union[float, np.ndarray], v_0: Union[float, np.ndarray], x_t: Union[float, np.ndarray],
                         v_t: Union[float, np.ndarray], t: float):
    a_0 = x_0
    a_1 = v_0
    a_2 = ((3 * (x_t - x_0)) - (((2 * v_0) + v_t) * t)) / (t ** 2)
    a_3 = ((2 * (x_0 - x_t)) + ((v_0 + v_t) * t)) / (t ** 3)
    return get_trajectory_fn(a_0, a_1, a_2, a_3)
