from copy import deepcopy
from threading import Lock

import rospy
from arc_utilities.ros_helpers import wait_for


class Listener:
    def __init__(self, topic_name, topic_type, wait_for_data=False, callback=None, queue_size=None):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.

        Listener does not consume the message
            (for consuming behavior, use the standard ros callback pattern)
        Listener does not check timestamps of message headers

        Parameters:
            topic_name (str):      name of topic to subscribe to
            topic_type (msg_type): type of message received on topic
            wait_for_data (bool):  block constructor until a message has been received
            callback (function taking msg_type): optional callback to be called on the data as we receive it
        """

        self.data = None
        self.lock = Lock()

        self.topic_name = topic_name
        self.subscriber = rospy.Subscriber(topic_name, topic_type, self.callback, queue_size=queue_size)
        self.custom_callback = callback
        self.get(wait_for_data)

    def callback(self, msg):
        with self.lock:
            self.data = msg
            if self.custom_callback is not None:
                self.custom_callback(self.data)

    def get(self, block_until_data=True):
        """
        Returns the latest msg from the subscribed topic

        Parameters:
            block_until_data (bool): block if no message has been received yet.
                                     Guarantees a msg is returned (not None)
        """
        wait_for(lambda: not (block_until_data and self.data is None), 10, f"Listener({self.topic_name})")

        with self.lock:
            return deepcopy(self.data)
