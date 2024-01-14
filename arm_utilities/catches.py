import signal
from typing import Callable, Optional


def _handle_timeout(_, __):
    raise TimeoutError()


def catch_timeout(seconds, func: Callable, *func_args, **func_kwargs):
    try:
        signal.signal(signal.SIGALRM, _handle_timeout)
        signal.alarm(seconds)
        try:
            result = func(*func_args, **func_kwargs)
            return result, False
        finally:
            signal.alarm(0)
    except TimeoutError:
        print("Caught timeout!")
        return None, True


def retry_on_timeout(seconds, on_timeout: Optional[Callable], gen: Callable, *gen_args, **gen_kwargs):
    """
    For generators
    Args:
        seconds: timeout in seconds
        gen: a generator, any function with `yield` or `field from`
        on_timeout: callback used when timeouts happen
    """
    it = gen(*gen_args, **gen_kwargs)
    while True:
        try:
            i, timed_out = catch_timeout(seconds, next, it)
            if timed_out:
                it = gen(*gen_args, **gen_kwargs)  # reset the generator
                if on_timeout is not None:
                    on_timeout()
            else:
                yield i
        except StopIteration:
            return


def skip_on_timeout(seconds, gen: Callable, *gen_args, **gen_kwargs):
    """
    For generators
    Args:
        seconds: timeout in seconds
        gen: a generator, any function with `yield` or `field from`
    """
    it = gen(*gen_args, **gen_kwargs)
    while True:
        try:
            i, timed_out = catch_timeout(seconds, next, it)
            if not timed_out:
                yield i
        except StopIteration:
            return
