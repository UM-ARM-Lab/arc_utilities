from typing import Iterator


def consecutive_pairs(iterator: Iterator):
    for first, second in zip(iterator, iterator[1:]):
        yield first, second
