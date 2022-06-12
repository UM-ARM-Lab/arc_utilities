from threading import Lock, Thread

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def pose_to_transform_stamped(pos, orn, parent_frame, child_frame, stamp=None):
    t = TransformStamped()
    if stamp is None:
        t.header.stamp = rospy.Time.now()
    else:
        t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]

    t.transform.rotation.x = orn[0]
    t.transform.rotation.y = orn[1]
    t.transform.rotation.z = orn[2]
    t.transform.rotation.w = orn[3]
    return t


class AsyncTFPublisher(object):

    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.done = False

        self.lock = Lock()
        self.transform_dict = dict()

        self.p = Thread(target=self._publish)
        self.p.start()

    def add_transform(self, frame_name: str, tf: TransformStamped):
        with self.lock:
            self.transform_dict[frame_name] = tf

    def add_transform_components(self, frame_name: str, parent_frame: str, child_frame: str, components):
        # Build transform.
        transform_stamped = pose_to_transform_stamped(components[0], components[1], parent_frame, child_frame)
        with self.lock:
            self.transform_dict[frame_name] = transform_stamped

    def add_transform_pose_array(self, frame_name: str, parent_frame: str, child_frame: str, pose_array):
        # Build transform.
        transform_stamped = pose_to_transform_stamped(pose_array[:3], pose_array[3:], parent_frame, child_frame)
        with self.lock:
            self.transform_dict[frame_name] = transform_stamped

    def close(self):
        with self.lock:
            self.done = True
        self.p.join(1.0)

    def _publish(self):
        while not rospy.is_shutdown():
            with self.lock:
                for _, tf in self.transform_dict.items():
                    tf.header.stamp = rospy.Time.now()
                    self.br.sendTransform(tf)
                if self.done:
                    return
            rospy.sleep(0.1)


class AsyncPublisher(object):

    def __init__(self, topic, msg_type, queue_size=10, hz=10.0):
        self.topic = topic
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.msg = None
        self.done = False
        self.hz = hz
        self.timestep = 1.0 / self.hz

        self.publisher = rospy.Publisher(self.topic, self.msg_type, queue_size=self.queue_size)
        self.lock = Lock()

        self.p = Thread(target=self._publish)
        self.p.start()

    def set_msg(self, msg):
        with self.lock:
            self.msg = msg

    def close(self):
        with self.lock:
            self.done = True
        self.p.join(1.0)

    def _publish(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.msg is not None:
                    if hasattr(self.msg, 'header'):
                        self.msg.header.stamp = rospy.Time.now()
                    self.publisher.publish(self.msg)
                if self.done:
                    return
            # Sleep outside of lock.
            rospy.sleep(self.timestep)
