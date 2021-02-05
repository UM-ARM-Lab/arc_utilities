#! /usr/bin/env python

import time
from typing import Optional

import rospy


def wait_for(func, warn_after: Optional[int] = 10, name: Optional[str] = ""):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """

    start_t = rospy.Time.now()
    has_warned = False

    while not func() and not rospy.is_shutdown():
        if warn_after is not None and rospy.Time.now() - start_t > rospy.Duration(secs=warn_after) and not has_warned:
            warning = f"still waiting after {warn_after}s"
            if name:
                warning += f" for {name}"
            rospy.logwarn_throttle(5, warning)
        time.sleep(0.01)


def joy_to_xbox(joy, xpad=True):
    """
    Transforms a joystick sensor_msg to a XBox controller for easier code readability

    Parameters:
    joy (sensor_msgs/Joy): xbox msg
    xpad (bool): True if using the default xpad driver, False if using xboxdrv

    Returns:
    xbox struct where fields are the button names
    """

    class Xbox_msg():
        def __str__(self):
            items = vars(self).items()
            return "\n".join("%s: %s" % item for item in items)

    x = Xbox_msg()
    if xpad:
        x.A, x.B, x.X, x.Y, x.LB, x.RB, \
        x.back, x.start, x.power, \
        x.stick_button_left, x.stick_button_right, \
        x.DL, x.DR, x.DU, x.DD = joy.buttons
        x.LH, x.LV, x.RH, x.RV, x.RT, x.LT, x.DH, x.DV = joy.axes
    else:
        x.A, x.B, x.X, x.Y, x.LB, x.RB, \
        x.back, x.start, x.power, \
        x.stick_button_left, x.stick_button_right = joy.buttons
        x.LH, x.LV, x.RH, x.RV, x.RT, x.LT, x.DH, x.DV = joy.axes
    return x


def logfatal(exception_class, msg):
    rospy.logfatal(msg)
    raise exception_class(msg)
