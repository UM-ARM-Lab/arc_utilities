from threading import Thread, Event
from time import sleep

from arm_utilities import ros_init


def target(exit_event):
    while not exit_event.is_set():
        print("running...")
        sleep(1.0)
    print("thread exiting.")


class Foo:
    def __init__(self):
        self.exit_event = Event()
        self.thread = Thread(target=target, args=(self.exit_event,))
        self.thread.start()

    def __del__(self):
        print("die!")
        self.exit_event.set()


@ros_init.with_ros("test_class_with_thread")
def main():
    Foo()
    print("Program ended...")


if __name__ == '__main__':
    main()
