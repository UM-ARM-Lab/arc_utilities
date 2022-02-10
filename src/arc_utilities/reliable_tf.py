from functools import partial
from threading import Thread, Event

import numpy as np

import ros_numpy
import rospy
import tf
from arc_utilities.tf2wrapper import TF2Wrapper
from geometry_msgs.msg import Pose
from rospy import Rate


def tfkey(*, parent, child):
    k = (parent, child)
    return k


def send_transform(exit_event, translation, quaternion, parent, child, is_static):
    tfw = TF2Wrapper()
    r = Rate(10)
    while not exit_event.is_set():
        tfw.send_transform(translation, quaternion, parent, child, is_static, time=rospy.Time.now())
        r.sleep()


def send_transform_matrix(exit_event, transform, parent, child, is_static):
    tfw = TF2Wrapper()
    r = Rate(10)
    while not exit_event.is_set():
        tfw.send_transform_matrix(transform, parent, child, is_static, time=rospy.Time.now())
        r.sleep()


def send_transform_from_pose_msg(exit_event, pose, parent, child, is_static):
    tfw = TF2Wrapper()
    r = Rate(10)
    while not exit_event.is_set():
        tfw.send_transform_from_pose_msg(pose, parent, child, is_static, time=rospy.Time.now())
        r.sleep()


class ReliableTF(TF2Wrapper):
    """
    abstracts the process of repeatedly sending transforms, to make it look more like sending transforms is guaranteed.
    this class is a foot gun, it will probably break if you use it in complicated ways.
    """

    def __init__(self):
        super().__init__()
        self.managed_tfs = {}

    def __del__(self):
        self.close()

    def close(self):
        for parent, child in list(self.managed_tfs.keys()):
            self.stop_send(parent, child)

    def stop_send(self, parent, child):
        k = tfkey(parent=parent, child=child)
        if k in self.managed_tfs:
            thread, exit_event = self.managed_tfs.pop(k)
            exit_event.set()
            thread.join()

    def start_send_transform_matrix(self, transform, parent, child, is_static=False):
        send_transform_partial = partial(send_transform_matrix, transform=transform, parent=parent,
                                         child=child, is_static=is_static)

        def _is_close_func(received_transform):
            return np.allclose(transform, received_transform)

        self.start_send(parent, child, _is_close_func, send_transform_partial)

    def start_send_transform_from_pose_msg(self, pose: Pose, parent, child, is_static=False):
        send_transform_partial = partial(send_transform_from_pose_msg, pose=pose, parent=parent,
                                         child=child, is_static=is_static)

        def _is_close_func(received_transform):
            return np.allclose(ros_numpy.numpify(pose), received_transform)

        self.start_send(parent, child, _is_close_func, send_transform_partial)

    def start_send_transform(self, translation, quaternion, parent, child, is_static=False):
        send_transform_partial = partial(send_transform, translation=translation, quaternion=quaternion, parent=parent,
                                         child=child, is_static=is_static)

        def _is_close_func(received):
            received_translation = received[:-1, -1]
            received_quaternion = tf.transformations.quaternion_from_matrix(received)
            trans_close = np.allclose(translation, received_translation)
            rot_close = np.allclose(quaternion, received_quaternion)
            return trans_close and rot_close

        self.start_send(parent, child, _is_close_func, send_transform_partial)

    def start_send(self, parent, child, is_close_func, send_transform_partial):
        exit_event = Event()

        k = tfkey(parent=parent, child=child)
        thread = Thread(target=send_transform_partial, args=(exit_event,))
        thread.start()

        self.managed_tfs[k] = (thread, exit_event)

        # block until the correct transform is received
        while True:
            received_transform = self.get_transform(parent, child, verbose=False)
            if is_close_func(received_transform):
                break
