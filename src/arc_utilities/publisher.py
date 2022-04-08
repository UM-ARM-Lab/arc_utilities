from threading import Lock, Thread

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class AsyncTFPublisher(object):

    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()

        self.lock = Lock()
        self.transform_dict = dict()

        self.p = Thread(target=self._publish)
        self.p.start()

    def add_transform(self, frame_name: str, tf: TransformStamped):
        with self.lock:
            self.transform_dict[frame_name] = tf

    def _publish(self):
        while not rospy.is_shutdown():
            with self.lock:
                for _, tf in self.transform_dict.items():
                    tf.header.stamp = rospy.Time.now()
                    self.br.sendTransform(tf)
            rospy.sleep(0.1)


class AsyncPublisher(object):

    def __init__(self, topic, msg_type, queue_size=10):
        self.topic = topic
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.msg = None

        self.publisher = rospy.Publisher(self.topic, self.msg_type, queue_size=self.queue_size)
        self.lock = Lock()

        self.p = Thread(target=self._publish)
        self.p.start()

    def set_msg(self, msg):
        with self.lock:
            self.msg = msg

    def _publish(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.msg is not None:
                    self.publisher.publish(self.msg)

            # Sleep outside of lock.
            rospy.sleep(0.1)
