#! /usr/bin/env python

import rospy
import time
from threading import Lock
from sensor_msgs.msg import Joy

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
        """

        self.data = None
        self.lock = Lock()
            
        self.subscriber = rospy.Subscriber(topic_name, topic_type, self.callback)
        

    def callback(self, msg):
        with self.lock:
            self.data = msg

    def get(self, block_on_none=False):
        """
        Returns the latest msg from the subscribed topic

        Parameters:
        block_on_none (bool): block return until data has been received
        """
        wait_for(lambda: not (block_on_none and self.data is None))
            
        with self.lock:
            return self.data

    
def wait_for(func):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """
    
    while not func() and not rospy.is_shutdown():
        time.sleep(0.01)

def joy_to_xbox(joy):
    """
    Transforms a joystick sensor_msg to a XBox controller for easier code readability
    
    Parameters:
    joy (sensor_msgs/Joy): xbox msg

    Returns:
    xbox struct where fields are the button names
    """
    class Xbox_msg():
        pass
    x = Xbox_msg()
    x.A, x.B, x.X, x.Y, x.LB, x.RB, \
        x.back, x.start, x.power,\
        x.stick_button_left, x.stick_button_right, \
        x.DL, x.DR, x.DU, x.DD = joy.buttons
    x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    return x
        


class Xbox():
    def __init__(self):
        self.xbox_listener = Listener("/joy", Joy)

    def get_buttons_state(self):
        return joy_to_xbox(self.xbox_listener.get(True))

    def wait_for_button(self, button, message=True):
        """
        Waits for button press on xbox.

        Parameters:
        button (str):   Name of xbox button. "A", "B", "X", ...
        message (bool): log a message informing the user?
        """
        if message:
            rospy.loginfo("Waiting for xbox button: " + button)
            
        wait_for(lambda: getattr(self.get_buttons_state(), button))
