#! /usr/bin/env python

import rospy
from threading import Lock


class Listener:
    def __init__(self, topic_name, topic_type, lock=None):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.

        Parameters:
            topic_name (str): name of topic to subscribe to
            topic_type (msg_type): type of message received on topic
            lock (Lock): optional lock object used when setting stored data
        """

        self.data = None
        self.lock = lock
        if self.lock is None:
            self.lock = Lock()
            
        self.subscriber = rospy.Subscriber(topic_name, topic_type, self.callback)
        

    def callback(self, msg):
        with self.lock:
            self.data = msg

    def get(self):
        """
        Returns the latest msg from the subscribed topic
        """
        with self.lock:
            return self.data
    
