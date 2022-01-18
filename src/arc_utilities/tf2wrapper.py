#! /usr/bin/env python

import rospy
import tf
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
import tf2_ros
from arc_utilities import transformation_helper
from geometry_msgs.msg import Pose, TransformStamped


class TF2Wrapper:
    def __init__(self):
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

    def get_transform(self,
                      parent,
                      child,
                      verbose=True,
                      spin_delay=rospy.Duration(secs=0, nsecs=500 * 1000 * 1000),
                      time=rospy.Time()):
        """
        Waits for a transform to become available. Blocks until a transform is available or an exception is raised.

        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param verbose: If verbose is True, then output messages are sent on rosinfo as the function waits for
                        a transform, otherwise on rosdebug
        :param spin_delay: How long to wait between output messages
        :param time: The timepoint to request a transform at. Defaults to "latest available".
        :return: A matrix representation of the transform (numpy). Returns None if a tf2 exception is raised.

        The notation here follows the following convention:

        p_measured_in_parent = returned_transform * p_measured_in_child
        p_measured_in_target = returned_transform * p_measured_in_source
        """

        try:
            transform = self.get_transform_msg(parent=parent, child=child, verbose=verbose, spin_delay=spin_delay,
                                               time=time)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("No transform available: %s to %s", parent, child)
            return None

        return transformation_helper.BuildMatrixRos(transform.transform.translation, transform.transform.rotation)

    def get_transform_msg(self,
                          parent,
                          child,
                          verbose=True,
                          spin_delay=rospy.Duration(secs=10, nsecs=0),
                          time=rospy.Time()):
        self.tf_listener.waitForTransform(parent, child, rospy.Time(), rospy.Duration(4))
        while not rospy.is_shutdown():
            rospy.sleep(spin_delay)
            try:
                now = rospy.Time.now()
                self.tf_listener.waitForTransform(parent, child, now, rospy.Duration(4))
                translate, quat = self.tf_listener.lookupTransform(parent, child, now)
                msg = TransformStamped()
                msg.transform.translation.x = translate[0]
                msg.transform.translation.y = translate[1]
                msg.transform.translation.z = translate[2]
                msg.transform.rotation.x = quat[0]
                msg.transform.rotation.y = quat[1]
                msg.transform.rotation.z = quat[2]
                msg.transform.rotation.w = quat[3]
                msg.header.stamp = now
                return msg
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e:
                if verbose:
                    print(e)

    def send_transform_matrix(self, transform, parent, child, is_static=False, time=None):
        """
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param transform: A matrix representation of the transform (presumably numpy)
        :param time: The timestamp for the transform, defaults to now()

        The notation here follows the following convention:

        p_measured_in_parent = transform * p_measured_in_child
        p_measured_in_target = transform * p_measured_in_source
        """
        [translation, quaternion] = transformation_helper.ExtractFromMatrix(transform)
        self.send_transform(translation, quaternion, parent, child, is_static, time)

    def send_transform_from_pose_msg(self, pose: Pose, parent, child, is_static=False, time=None):
        if time is None:
            time = rospy.Time.now()

        self.tf_broadcaster.sendTransform(pose.position, pose.orientation, time, child, parent)

    def send_transform(self, translation, quaternion, parent, child, is_static=False, time=None):
        """
        :param parent: frame name for the parent (see below)
        :param child: frame name for the child (see below)
        :param translation: [x, y, z]
        :param quaternion: [x, y, z, w]
        :param time: The timestamp for the transform, defaults to now()

        The notation here follows the following convention:

        p_measured_in_parent = transform * p_measured_in_child
        p_measured_in_target = transform * p_measured_in_source
        """
        if time is None:
            time = rospy.Time.now()

        self.tf_broadcaster.sendTransform(translation, quaternion, time, child, parent)

    def transform_to_frame(self, object_stamped, target_frame, timeout=rospy.Duration(0), new_type=None):
        """
        Transforms many "stamped" data types between frames. The specific package for the type of stamped object needs
         to be imported prior to use. Examples are tf2_geometry_msgs and tf2_py.
        If new_type is not None, the type specified must have a valid conversion from the input type, else the function
         will raise an exception.
        Example usage:
            from arc_utilities import ros_helpers
            import tf2_geometry_msgs
            ...
            self.tf2 = ros_helpers.TF2Wrapper()
            ...
            p_in_native_frame = PointStamped()
            p_in_native_frame.header.stamp = rospy.Time.now() # This will likely cause an extrapolation warning/exception without a timeout set
            p_in_native_frame.header.frame_id = frame_point_is_measured_in
            p_in_native_frame.point = ...
            p_in_world = self.tf2.transform_to_frame(object_stamped=p_in_native_frame, target_frame=world_frame_name)

        :param object_stamped: The timestamped object the transform.
        :param target_frame: Name of the frame to transform the input into.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param new_type: (Optional) Type to convert the object to.
        :return: The transformed, timestamped output, possibly converted to a new type.
        """
        return self.tf2_buffer.transform(object_stamped, target_frame, timeout, new_type)
