#! /usr/bin/env python

import rospy
from threading import Lock


class Listener:
    def __init__(self, topic_name, topic_type):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.

        Listener does not consume the message 
            (for consuming behavior, use the standard ros callback pattern)
        Listener does not check timestamps of message headers

        Parameters:
            topic_name (str): name of topic to subscribe to
            topic_type (msg_type): type of message received on topic
            lock (Lock): optional lock object used when setting stored data
        """

        self.data = None
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


def joy_to_xbox(joy):
    """
    Transforms a joystick sensor_msg to a XBox controller for easier code readability
    
    Parameters:
    joy (sensor_msgs/Joy): xbox msg

    Returns:
    xbox struct where fields are the button names
    """
    x = object
    x.A, x.B, x.X, x.Y, x.LB, x.RB, \
        x.back, x.start, x.power,\
        x.stick_button_left, x.stick_button_right, \
        x.DL, x.DR, x.DU, x.DD = joy.buttons
    x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    return x
        

    
