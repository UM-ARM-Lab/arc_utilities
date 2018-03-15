#! /usr/bin/env python

import rospy
import time
from copy import deepcopy
from threading import Lock
from sensor_msgs.msg import Joy
import tf2_ros
import geometry_msgs.msg
from arc_utilities import transformation_helper

class Listener:
    def __init__(self, topic_name, topic_type, wait_for_data=False):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.

        Listener does not consume the message 
            (for consuming behavior, use the standard ros callback pattern)
        Listener does not check timestamps of message headers

        Parameters:
            topic_name (str):      name of topic to subscribe to
            topic_type (msg_type): type of message received on topic
            wait_for_data (bool):  block constructor until a message has been received
        """

        self.data = None
        self.lock = Lock()
            
        self.subscriber = rospy.Subscriber(topic_name, topic_type, self.callback)
        self.get(wait_for_data)

    def callback(self, msg):
        with self.lock:
            self.data = msg

    def get(self, block_until_data=True):
        """
        Returns the latest msg from the subscribed topic

        Parameters:
            block_until_data (bool): block if no message has been received yet. 
                                     Guarantees a msg is returned (not None)
        """
        wait_for(lambda: not (block_until_data and self.data is None))
            
        with self.lock:
            return deepcopy(self.data)

    
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
    def __init__(self, joystick_topic="joy"):
        self.xbox_listener = Listener(joystick_topic, Joy)

    def get_buttons_state(self):
        """
        Returns an xbox struct of the last joystick message received
        """
        return joy_to_xbox(self.xbox_listener.get())

    def get_button(self, button):
        """
        Return value of button or axis of the controller
        0 or 1 for buttons
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


class TF2Wrapper:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcasters = []

    def get_tf_transform(self, parent, child, verbose=False):
        """
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)

        The notation here follows the following convention:

        p_measured_in_parent = returned_transform * p_measured_in_child
        p_measured_in_target = returned_transform * p_measured_in_source

        :param verbose:
        :return: A matrix representation of the transform (numpy)
        """
        try:
            # Wait for 0.5 seconds every loop
            timeout = rospy.Duration(secs=0, nsecs=500 * 1000 * 1000)

            # Get the latest available transform
            latest_available = rospy.Time()

            while not self.tf_buffer.can_transform(child, parent, time=latest_available, timeout=timeout):
                if rospy.is_shutdown():
                    raise KeyboardInterrupt("ROS has shutdown")
                print "Waiting for TF frames ", parent, " and ", child
            transform = self.tf_buffer.lookup_transform(parent, child, time=latest_available)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No transform available: %s to %s", parent, child)
            return

        return transformation_helper.BuildMatrixRos(transform.transform.translation, transform.transform.rotation)

    def send_tf_transform(self, transform, parent, child, is_static=False):
        """
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param transform: A matrix representation of the transform (presumably numpy)

        The notation here follows the following convention:

        p_measured_in_parent = transform * p_measured_in_child
        p_measured_in_target = transform * p_measured_in_source
        """
        [translation, quaternion] = transformation_helper.ExtractFromMatrix(transform)
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        if is_static:
            self.tf_static_broadcasters.append(tf2_ros.StaticTransformBroadcaster())
            self.tf_static_broadcasters[-1].sendTransform(t)
        else:
            self.tf_broadcaster.sendTransform(t)

    def transform_to_frame(self, object_stamped, target_frame, timeout=rospy.Duration(0.0), new_type=None):
        """
        Transforms many "stamped" data types between frames. The specific package for the type of stamped object needs
         to be imported prior to use. Examples are tf2_geometry_msgs and tf2_py.
        If new_type is not None, the type specified must have a valid conversion from the input type, else the function
         will raise an exception.
        Example usage:
            import tf2_geometry_msgs
            ...
            p_in_native_frame = PointStamped()
            p_in_native_frame.header.stamp = rospy.Time.now() # This will likely cause an extrapolation warning/exception without a timeout set
            p_in_native_frame.header.frame_id = frame_point_is_measured_in
            p_in_native_frame.point = ...
            p_in_world = self.planner.transform_to_frame(object_stamped=p_in_native_frame, target_frame=world_frame_name)
        :param object_stamped: The timestamped object the transform.
        :param target_frame: Name of the frame to transform the input into.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param new_type: (Optional) Type to convert the object to.
        :return: The transformed, timestamped output, possibly converted to a new type.
        """
        return self.tf_buffer.transform(object_stamped, target_frame, timeout, new_type)