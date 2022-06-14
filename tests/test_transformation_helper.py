import unittest

import numpy as np
import tf.transformations as tf
from arc_utilities.transformation_helper import vector3_to_spherical, spherical_to_vector3, ChangeTransformFrame, \
    BuildMatrix
import arc_utilities.transformation_helper as tf_helper


def get_random_transform():
    trans = np.random.random(3)
    rot = tf.random_quaternion()
    return trans, rot


class TestTransformationHelper(unittest.TestCase):

    def test_vector3_to_spherical_zero(self):
        xyz = [0, 0, 0]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case1(self):
        xyz = [1, 0, 0]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case2(self):
        xyz = [0, 1, 0]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case3(self):
        xyz = [0, 0, 1]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_case4(self):
        xyz = [0, 1, 1]
        r_phi_theta = vector3_to_spherical(xyz)
        xyz_out = spherical_to_vector3(r_phi_theta)
        np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_vector3_to_spherical_random(self):
        rng = np.random.RandomState(0)
        for i in range(100):
            xyz = rng.uniform(-2, 1, size=[3])
            r_phi_theta = vector3_to_spherical(xyz)
            xyz_out = spherical_to_vector3(r_phi_theta)
            np.testing.assert_allclose(xyz, xyz_out, rtol=1)

    def test_change_transform_frame(self):
        np.random.seed(42)
        for _ in range(100):
            tf_0_A = BuildMatrix(*get_random_transform())
            tf_0_T_1 = BuildMatrix(*get_random_transform())
            tf_1_T_0 = np.linalg.inv(tf_0_T_1)
            tf_0_P_2 = BuildMatrix(*get_random_transform())
            tf_1_P_2 = tf_1_T_0 @ tf_0_P_2

            tf_0_P_3 = tf_0_A @ tf_0_P_2

            tf_1_A = tf_helper.TransformToMatrix(ChangeTransformFrame(tf_helper.TransformFromMatrix(tf_0_A),
                                                                      tf_helper.TransformFromMatrix(tf_0_T_1)))
            tf_1_P_3 = tf_1_A @ tf_1_P_2

            self.assertTrue(np.allclose(tf_0_P_3, tf_0_T_1 @ tf_1_P_3))


if __name__ == '__main__':
    np.set_printoptions(suppress=True, precision=6)
    unittest.main()
