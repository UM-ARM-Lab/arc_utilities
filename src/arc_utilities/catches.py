import signal
from typing import Callable, Optional


def _handle_timeout(_, __):
    raise TimeoutError()


def catch_timeout(seconds, func: Callable, *args, **kwargs):
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


def retry_on_timeout(seconds, on_timeout: Optional[Callable], gen: Callable, *args, **kwargs):
    """
    For generators
    Args:
        seconds: timeout in seconds
        gen: a generator, any function with `yield` or `field from`
        on_timeout: callback used when timeouts happen
    """
    it = gen(*args, **kwargs)
    while True:
        try:
            i, timed_out = catch_timeout(seconds, next, it)
            if timed_out:
                it = gen(*args, **kwargs)  # reset the generator
                if on_timeout is not None:
                    on_timeout()
            else:
                yield i
        except StopIteration:
            return


def skip_on_timeout(seconds, gen: Callable, *args, **kwargs):
    """
    For generators
    Args:
        seconds: timeout in seconds
        gen: a generator, any function with `yield` or `field from`
    """
    it = gen(*args, **kwargs)
    while True:
        try:
            i, timed_out = catch_timeout(seconds, next, it)
            if not timed_out:
                yield i
        except StopIteration:
            return
