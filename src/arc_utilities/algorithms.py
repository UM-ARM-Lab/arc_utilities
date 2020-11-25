import collections.abc


def update(d, u):
    """
    Update a nested dictionary or similar mapping.
    Modifies d in place.
    https://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth
    """
    for k, v in u.items():
        if isinstance(v, collections.abc.Mapping):
            d[k] = update(d.get(k, {}), v)
        else:
            d[k] = v
    return d


def is_list_unique(x):
    """ all elements must be hashable """
    return len(x) == len(set(x))
