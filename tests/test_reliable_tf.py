import unittest

import numpy as np

import rospy
from arc_utilities import ros_init
from arc_utilities.catches import catch_timeout
from arc_utilities.reliable_tf import ReliableTF
from arc_utilities.tf2wrapper import TF2Wrapper
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
    tf_1.start_send_transform([0, 0, 1.], [0, 0, 0, 1.], parent='world', child='a')
    tf_1.start_send_transform([0, 1., 1.], [0, 0, 0, 1], parent='world', child='b')

    tf_2 = ReliableTF()
    tf_2.start_send_transform([0, 2., 1.], [0, 0, 0, 1], parent='world', child='c')


def changing_transforms():
    expected1 = np.array([[1, 0, 0, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1.]])
    expected2 = np.array([[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1.]])
    tf = ReliableTF()
    tf.start_send_transform_matrix(expected1, parent='world', child='a')
    get1 = tf.get_transform(parent='world', child='a')
    np.testing.assert_allclose(get1, expected1)
    tf.start_send_transform_matrix(expected2, parent='world', child='a')
    get2 = tf.get_transform(parent='world', child='a')
    np.testing.assert_allclose(get2, expected2)


def reliable_tf_rocks():
    tf = ReliableTF()
    tf.start_send_transform([0, 0, 1.], [0, 0, 0, 1.], parent='world', child='a')
    tf.start_send_transform([0, 1., 1.], [0, 0, 0, 1], parent='world', child='b')
    pose = Pose()
    pose.position.x = 1
    pose.orientation.z = 1
    tf.start_send_transform_from_pose_msg(pose, parent='world', child='c')
    transform = np.array([
        [1, 0, 0, 1.],
        [0, 1, 0, 1.],
        [0, 0, 1, 1.],
        [0, 0, 0, 1.],
    ])
    tf.start_send_transform_matrix(transform, parent='world', child='d')

    def _get():
        return tf.get_transform(parent='a', child='b')

    a2b, timeout = catch_timeout(1, _get)

    return a2b, timeout


@ros_init.with_ros("test_reliable_tf")
def main():
    # show that tf doesn't work reliably when you just publish one transform
    # tf_sucks()
    # reliable_tf_rocks()
    changing_transforms()
    # test_multiple_instances()
    # tf = TF2Wrapper()


class TestReliableTF(unittest.TestCase):

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
    # main()
    rospy.init_node("test_reliable_tf")
    unittest.main()