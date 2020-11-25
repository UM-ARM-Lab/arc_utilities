import asyncio
from threading import Timer
from typing import Type

import rospy


class Repeater:

    def __init__(self, topic_name: str, type: Type, period: float = 1.0, queue_size: int = 10):
        """
        @period publish the latest message every this many seconds
        """
        self.period = period
        self.pub = rospy.Publisher(topic_name, type, queue_size=queue_size)
        self.timer = Timer(self.period, self.run)
        self.lock = asyncio.Lock()

    def publish(self, msg):
        with self.lock:
            self.latest_msg = msg

    def run(self):
        with self.lock:
            msg = self.latest_msg
        # this way we don't need to wait for 'publish' before releasing the lock
        self.pub.publish(msg)
