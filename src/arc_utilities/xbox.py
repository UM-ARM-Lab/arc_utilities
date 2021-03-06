import numpy as np
from inputs import InputEvent

import rospy
from arc_utilities.listener import Listener
from arc_utilities.ros_helpers import joy_to_xbox, wait_for
from sensor_msgs.msg import Joy


def deadzone(x):
    if abs(x) < 0.05:
        return 0
    return x


class Xbox():
    def __init__(self, joystick_topic="joy", xpad=True):
        """
        Parameters:
        joystick_topic (string): topic name to subscribe to
        xpad (bool): True if using the default xpad driver, False if using xboxdrv

        """
        self.xpad = xpad
        self.xbox_listener = Listener(joystick_topic, Joy)

    def get_buttons_state(self):
        """
        Returns an xbox struct of the last joystick message received
        """
        return joy_to_xbox(self.xbox_listener.get(), self.xpad)

    def get_button(self, button):
        """
        Return value of button or axis of the controller
        0 or 1 for button
        -1.0 to 1.0 (at most) for axes
        """
        return getattr(self.get_buttons_state(), button)

    def wait_for_button(self, button, message=True):
        """
        Waits for button press on xbox.

        Parameters:
        button (str):   Name of xbox button. "A", "B", "X", ...
        message (bool): log a message informing the user?
        """
        if message:
            rospy.loginfo("Waiting for xbox button: " + button)

        wait_for(lambda: not self.get_button(button) == 0)

    def x_clicked(self, event: InputEvent):
        return event.ev_type == 'Key' and event.code == 'BTN_NORTH' and event.state == 0

    def get_axis_normalized(self, axis: int):
        joy_msg = self.xbox_listener.get()
        if axis == 0:
            return deadzone(-joy_msg.axes[axis])
        elif axis == 1:
            return deadzone(joy_msg.axes[axis])
        elif axis == 2:
            return deadzone(joy_msg.axes[axis])
        elif axis == 4:
            return deadzone(joy_msg.axes[axis])
        elif axis == 5:
            return deadzone(joy_msg.axes[axis])
        else:
            raise NotImplementedError(f"axis {axis} is not implemented")

    def get_3d_delta(self):
        """
        We use the dpad for x,y and the left joystick to do z.
        NOTE: this is hardcoded for one of the several xbox linux drivers that exist,
        so if you change drivers this might not work.

        Returns:
            the [x, y, z], in the interval [-1.0, 1.0]

        """
        return np.array([self.get_axis_normalized(0),
                         self.get_axis_normalized(1),
                         self.get_axis_normalized(4)])
