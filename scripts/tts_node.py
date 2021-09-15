#!/usr/bin/env python
import subprocess

import rospy
from std_msgs.msg import String


def cb(msg: String):
    args = ["espeak", msg.data]
    subprocess.run(args)


def main():
    rospy.init_node("tts_node")
    sub = rospy.Subscriber("speak", String, cb)
    rospy.spin()


if __name__ == '__main__':
    main()
