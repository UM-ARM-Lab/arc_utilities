import collections.abc
import itertools
from typing import List, Dict, Optional


def nested_dict_update(base_dict: Dict, update_dict: Optional[Dict]):
    """
    Update a nested dictionary or similar mapping.
    Modifies d in place.
    https://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth
    """
    if update_dict is None:
        return base_dict
    for k, v in update_dict.items():
        if isinstance(v, collections.abc.Mapping):
            base_dict[k] = nested_dict_update(base_dict.get(k, {}), v)
        else:
            base_dict[k] = v
    return base_dict


def is_list_unique(x):
    """ all elements must be hashable """
    return len(x) == len(set(x))


def repeat_last(x: List):
    if len(x) == 0:
        raise RuntimeError("Cannot repeat the last element of an empty list")
    x_i = None
    for x_i in x:
        yield x_i
    while True:
        yield x_i


def zip_repeat_shorter(*args):
    longest_len = max(*[len(x) for x in args])
    zipped = zip(*[repeat_last(x) for x in args])
    for i in range(longest_len):
        yield next(zipped)


def chunked_iterable(iterable, size):
    """

    Args:
        iterable: an iterable to split into chunks. could be a list, tuple, or generator
        size: the size of the chunks. If the iterable cannot be evenly split, the final elements will be dropped.

    Returns: a generator of lists

    """
    it = iter(iterable)
    while True:
        chunk = list(itertools.islice(it, size))
        if not chunk:
            break
        if len(chunk) != size:
            break
        yield chunk


def reversed_chunked(seq, n: int):
    """
    Args:
        seq: some indexable sequence
        n: the size of the chunks

    Returns: A list of chunks, each chunk is of size n, if the seq is not divisible by n,
        then elements at the START of seq are discarded.

    """
    chunks = []
    for end in range(len(seq), n - 1, -n):
        start = end - n
        chunks.insert(0, seq[start:end])
    return chunks
