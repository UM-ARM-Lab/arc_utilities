import traceback
from types import TracebackType
from typing import Callable

import roscpp_initializer

import rospy


def rospy_and_cpp_init(name):
    """ Use this function any time you want to call into C++ ROS code.
      You can use this in place of the moveit roscpp_initializer, and """
    roscpp_initializer.init_node("cpp_" + name, [], disable_signals=True)
    rospy.init_node(name)


def shutdown():
    """ ensures the C++ node handle is shut down cleanly. It's good to call this a the end of any program
      where you called rospy_and_cpp_init """
    roscpp_initializer.shutdown()


class RosContext:
    def __init__(self, name):
        rospy_and_cpp_init(name)

    def __enter__(self):
        pass

    def __exit__(self, type, value, tb: TracebackType):
        if type:
            traceback.print_exception(type, value, tb)
        shutdown()
        return True


def with_ros(name: str):
    """
    A decorator for managing allocation an de-allocation of ROS resources. EXAMPLE:

    @with_ros
    def main():
      rospy.Publisher(...)

    Args:
        func: the function that gets wrapped with this decorator. It can be arbitrary , but it's usually "main"
        name: the ros node name

    Returns:
        This function returns a function which takes a function and returns a function that calls a function.
        Don't think too hard about it

    """

    def _with_ros(func: Callable):
        def wrapper(*args, **kwargs):
            with RosContext(name):
                func(*args, **kwargs)

        return wrapper

    return _with_ros
