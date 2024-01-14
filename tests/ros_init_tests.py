#! /usr/bin/env python
import time
import unittest

import rosnode
from arm_utilities import ros_init


class TestRosInit(unittest.TestCase):

    def test_ros_init(self):
        name = "test_node_name_" + str(int(time.time()))

        names = rosnode.get_node_names()
        self.assertNotIn("/" + name, names)

        ros_init.rclpy_and_cpp_init(name)
        time.sleep(1)

        names = rosnode.get_node_names()
        self.assertIn("/" + name, names)
        self.assertIn("/cpp_" + name, names)

        ros_init.shutdown()

        time.sleep(1)
        # this will still exist, shutdown only shut's down the internal C++ node
        # WRONG --> self.assertNotIn("/" + name, rosnode.get_node_names())
        self.assertNotIn("/cpp_" + name, rosnode.get_node_names())


if __name__ == '__main__':
    unittest.main()
