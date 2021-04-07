#! /usr/bin/env python

import time
from typing import Optional, Type

import rosgraph
import rospy


def wait_for(func, warn_after: Optional[int] = 10, name: Optional[str] = ""):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """

    start_t = rospy.Time.now()

    while not func() and not rospy.is_shutdown():
        if warn_after is not None and rospy.Time.now() - start_t > rospy.Duration(secs=warn_after):
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


def get_connected_publisher(topic_path: str, data_class: Type, *args, **kwargs):
    pub = rospy.Publisher(topic_path, data_class, *args, **kwargs)
    num_subs = len(_get_subscribers(topic_path))
    for i in range(10):
        num_cons = pub.get_num_connections()
        if num_cons == num_subs:
            return pub
        time.sleep(0.1)

    raise RuntimeError(f"failed to get publisher for {topic_path}")


def _get_subscribers(topic_path: str):
    ros_master = rosgraph.Master('/rostopic')
    topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
    state = ros_master.getSystemState()
    subs = []
    for sub in state[1]:
        if sub[0] == topic_path:
            subs.extend(sub[1])
    return subs


def try_to_connect(*publishers, raise_on_fail: bool = False):
    for i in range(10):
        connected = [p.get_num_connections() > 0 for p in publishers]
        if all(connected):
            return
        time.sleep(0.1)

    unconnected_pubs = [p.name for p in publishers if p.get_num_connections() == 0]
    if len(unconnected_pubs) > 0:
        msg = f"failed to connect publishers {','.join(unconnected_pubs)}"
        if raise_on_fail:
            raise RuntimeError(msg)
        rospy.logwarn(msg)
