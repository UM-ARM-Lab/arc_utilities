import unittest

import numpy as np

from arc_utilities.transformation_helper import vector3_to_spherical, spherical_to_vector3


class TestTransformationHelper(unittest.TestCase):

    def test_vector3_to_spherical_case1(self):
        v = [1, 0, 0]
        r, phi, theta = vector3_to_spherical(v)
        v_out = spherical_to_vector3(r, phi, theta)
        np.testing.assert_allclose(v, v_out, rtol=1)

    def test_vector3_to_spherical_case2(self):
        v = [0, 1, 0]
        r, phi, theta = vector3_to_spherical(v)
        v_out = spherical_to_vector3(r, phi, theta)
        np.testing.assert_allclose(v, v_out, rtol=1)

    def test_vector3_to_spherical_case3(self):
        v = [0, 0, 1]
        r, phi, theta = vector3_to_spherical(v)
        v_out = spherical_to_vector3(r, phi, theta)
        np.testing.assert_allclose(v, v_out, rtol=1)

    def test_vector3_to_spherical_case4(self):
        v = [0, 1, 1]
        r, phi, theta = vector3_to_spherical(v)
        v_out = spherical_to_vector3(r, phi, theta)
        np.testing.assert_allclose(v, v_out, rtol=1)

    def test_vector3_to_spherical_random(self):
        rng = np.random.RandomState(0)
        for i in range(100):
            v = rng.uniform(-2, 1, size=[3])
            r, phi, theta = vector3_to_spherical(v)
            v_out = spherical_to_vector3(r, phi, theta)
            np.testing.assert_allclose(v, v_out, rtol=1)


if __name__ == '__main__':
    np.set_printoptions(suppress=True, precision=6)
    unittest.main()
