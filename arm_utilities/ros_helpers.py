#! /usr/bin/env python
import time
from typing import Optional

import rclpy
import rclpy.logging
from sensor_msgs.msg import Joy

logger = rclpy.logging.get_logger("ros_helpers")


def joy_to_xbox(joy: Joy, xpad: bool = True):
    """
    Transforms a joystick sensor_msg to a XBox controller for easier code readability

    Parameters:
    joy: xbox msg
    xpad: True if using the default xpad driver, False if using xboxdrv

    Returns:
    xbox struct where fields are the button names
    """

    class Xbox_msg():
        def __str__(self):
            items = vars(self).items()
            return "\n".join("%s: %s" % item for item in items)

    x = Xbox_msg()
    if xpad:
        x.A, x.B, x.X, x.Y, x.LB, x.RB, x.back, x.start, x.power, x.stick_button_left, x.stick_button_right, x.DL, x.DR, x.DU, x.DD = joy.buttons
        x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    else:
        x.A, x.B, x.X, x.Y, x.LB, x.RB, x.back, x.start, x.power, x.stick_button_left, x.stick_button_right = joy.buttons
        x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    return x


def wait_for(func, warn_after: Optional[int] = 10, name: Optional[str] = ""):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """

    start_t = time.time()

    while not func() and not rclpy.ok():
        if warn_after is not None and time.time() - start_t > warn_after:
            warning = f"still waiting after {warn_after}s"
            if name:
                warning += f" for {name}"
            logger.warn(warning, throttle_duration_sec=5.0)
        time.sleep(0.01)
