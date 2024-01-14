import unittest

import numpy as np

import rclpy
from arm_utilities import ros_init
from arm_utilities.catches import catch_timeout
from arm_utilities.reliable_tf import ReliableTF
from arm_utilities.tf2wrapper import TF2Wrapper
from geometry_msgs.msg import Pose


@ros_init.with_ros("test_reliable_tf")
def tf_sucks():
    tf = TF2Wrapper()
    tf.send_transform([0, 0, 1.], [0, 0, 0, 1.], parent='world', child='a')
    tf.send_transform([0, 0, 1.], [0, 0, 0, 1], parent='world', child='b')

    def _get():
        return tf.get_transform(parent='a', child='b')

    a2b, timeout = catch_timeout(1, _get)
    return a2b, timeout


def test_multiple_instances():
    tf_1 = ReliableTF()
    tf_1.start_send_transform([0, 0, 1.], [0, 0, 0, 1.], parent='world', child='c')
    tf_1.start_send_transform([0, 1., 1.], [0, 0, 0, 1], parent='world', child='d')

    tf_2 = ReliableTF()
    tf_2.start_send_transform([0, 2., 1.], [0, 0, 0, 1], parent='world', child='e')


def changing_transforms():
    expected1 = np.array([[1, 0, 0, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1.]])
    expected2 = np.array([[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1.]])
    tf = ReliableTF()
    tf.start_send_transform_matrix(expected1, parent='world', child='f')
    get1 = tf.get_transform(parent='world', child='f')
    np.testing.assert_allclose(get1, expected1)
    tf.start_send_transform_matrix(expected2, parent='world', child='g')
    get2 = tf.get_transform(parent='world', child='g')
    np.testing.assert_allclose(get2, expected2)


def reliable_tf_rocks():
    tf = ReliableTF()
    tf.start_send_transform([0, 0, 1.], [0, 0, 0, 1.], parent='world', child='h')
    tf.start_send_transform([0, 1., 1.], [0, 0, 0, 1], parent='world', child='i')
    pose = Pose()
    pose.position.x = 1
    pose.orientation.z = 1
    tf.start_send_transform_from_pose_msg(pose, parent='world', child='j')
    transform = np.array([
        [1, 0, 0, 1.],
        [0, 1, 0, 1.],
        [0, 0, 1, 1.],
        [0, 0, 0, 1.],
    ])
    tf.start_send_transform_matrix(transform, parent='world', child='k')

    def _get():
        return tf.get_transform(parent='h', child='i')

    a2b, timeout = catch_timeout(1, _get)

    return a2b, timeout


class TestReliableTF(unittest.TestCase):
    def setUp(self):
        rclpy.init_node("test_reliable_tf")

    def test_tf_sucks(self):
        a2b, timeout = tf_sucks()
        self.assertIsNone(a2b)
        self.assertTrue(timeout)

    def test_reliable_tf_rocks(self):
        a2b, timeout = reliable_tf_rocks()
        self.assertIsNotNone(a2b)
        np.testing.assert_allclose(a2b, np.array([[1, 0, 0, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1.]]))
        self.assertFalse(timeout)

    def test_changing_transform(self):
        changing_transforms()


if __name__ == '__main__':
    unittest.main()
