import unittest

import matplotlib.pyplot as plt
import numpy as np
from arc_utilities.trajectory_planning import get_linear_trajectory, get_quadratic_trajectory, get_cubic_trajectory


class TestTrajectoryPlanning(unittest.TestCase):

    def test_linear(self):
        x_0 = 0.0
        v_0 = 1.0
        traj_fn = get_linear_trajectory(x_0, v_0)

        x_t, v_t, a_t = traj_fn(t=0.5)
        self.assertEqual(x_t, 0.5)
        self.assertEqual(v_t, v_0)
        self.assertEqual(a_t, 0.0)

        x_t, v_t, a_t = traj_fn(t=2.0)
        self.assertEqual(x_t, 2.0)
        self.assertEqual(v_t, v_0)
        self.assertEqual(a_t, 0.0)

    def test_quadratic(self):
        x_0 = 0.0
        v_0 = 1.0
        x_1 = 5.0
        t = 2.0

        traj_fn = get_quadratic_trajectory(x_0, v_0, x_1, t)

        x_t, v_t, a_t = traj_fn(t=0.0)
        self.assertEqual(x_t, 0.0)
        self.assertEqual(v_t, v_0)
        self.assertEqual(a_t, 1.5)

        x_t, v_t, a_t = traj_fn(t=1.0)
        self.assertEqual(x_t, 1.75)
        self.assertEqual(v_t, 2.5)
        self.assertEqual(a_t, 1.5)

        x_t, v_t, a_t = traj_fn(t=2.0)
        self.assertEqual(x_t, 5.0)
        self.assertEqual(v_t, 4.0)
        self.assertEqual(a_t, 1.5)

    def test_cubic(self):
        x_0 = 0.0
        v_0 = 0.0
        x_1 = 5.0
        v_1 = 0.0
        t = 2.0

        traj_fn = get_cubic_trajectory(x_0, v_0, x_1, v_1, t)

        x_t, v_t, a_t = traj_fn(t=0.0)
        self.assertEqual(x_t, x_0)
        self.assertEqual(v_t, v_0)

        x_t, v_t, a_t = traj_fn(t=t)
        self.assertEqual(x_t, x_1)
        self.assertEqual(v_t, v_1)

    def test_cubic_multidim(self):
        x_0 = np.zeros(4)
        v_0 = np.zeros(4)
        x_1 = np.array([0.5, -0.1, 3.0, 0.1])
        v_1 = np.zeros(4)
        t = 3.0

        traj_fn = get_cubic_trajectory(x_0, v_0, x_1, v_1, t)

        x_t, v_t, a_t = traj_fn(t=0.0)
        self.assertTrue(np.allclose(x_t, x_0))
        self.assertTrue(np.allclose(v_t, v_0))

        x_t, v_t, a_t = traj_fn(t=t)
        self.assertTrue(np.allclose(x_t, x_1))
        self.assertTrue(np.allclose(v_t, v_1))


if __name__ == '__main__':
    unittest.main()
