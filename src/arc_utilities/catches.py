import signal
from typing import Callable, Optional


def catch_timeout(seconds: int, func: Callable, *args, **kwargs):
    def _handle_timeout(signum, frame):
        raise TimeoutError()

    try:
        signal.signal(signal.SIGALRM, _handle_timeout)
        signal.alarm(seconds)
        try:
            result = func(*args, **kwargs)
            return result, False
        finally:
            signal.alarm(0)
    except TimeoutError:
        print("Caught timeout!")
        return None, True


def retry_on_timeout(t: int, on_timeout: Optional[Callable], f: Callable, *args, **kwargs):
    """
    For generators
    Args:
        t: timeout in seconds
        f: a generator, any function with `yield` or `field from`
        on_timeout: callback used when timeouts happen

    Returns:

    """
    it = f(*args, **kwargs)
    while True:
        try:
            i, timed_out = catch_timeout(t, next, it)
            if timed_out:
                it = f(*args, **kwargs)  # reset the generator
                if on_timeout is not None:
                    on_timeout()
            else:
                yield i
        except StopIteration:
            return


def skip_on_timeout(t: int, on_timeout: Optional[Callable], f: Callable, *args, **kwargs):
    """
    For generators
    Args:
        t: timeout in seconds
        f: a generator, any function with `yield` or `field from`
        on_timeout: callback used when timeouts happen

    Returns:

    """
    it = f(*args, **kwargs)
    while True:
        try:
            i, timed_out = catch_timeout(t, next, it)
            if not timed_out:
                yield i
        except StopIteration:
            return
