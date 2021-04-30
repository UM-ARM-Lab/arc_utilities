#! /usr/bin/env python
import unittest

import rospy
from arc_utilities.ros_helpers import try_to_connect
from std_msgs.msg import Empty


def cb(msg):
    pass


class TestCase(unittest.TestCase):

    def test_try_connect(self):
        rospy.init_node("testing")
        topic_name = "/testing"
        pub = rospy.Publisher(topic_name, Empty, queue_size=10)
        sub = rospy.Subscriber(topic_name, Empty, callback=cb)
        try_to_connect(pub)
        self.assertEqual(pub.get_num_connections(), 1)

    def test_try_connect_fail(self):
        topic_name = "/testing_fail"
        pub = rospy.Publisher(topic_name, Empty, queue_size=10)
        try_to_connect(pub)
        self.assertEqual(pub.get_num_connections(), 0)

        with self.assertRaises(RuntimeError):
            try_to_connect(pub, raise_on_fail=True)
        self.assertEqual(pub.get_num_connections(), 0)


if __name__ == '__main__':
    unittest.main()
