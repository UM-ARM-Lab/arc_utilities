#! /usr/bin/env python

"""
Useful functions for dealing with paths
"""

import IPython
import numpy as np

def clamp(num, min_val, max_val):
    return min(max(min_val, num), max_val)

def dist(p1, p2):
    return np.linalg.norm(np.array(p1)-np.array(p2))

def closest_point_to_line(line, point):
    """
    Returns:
    point, alpha
    alpha: 0 to 1
    """
    v_line = np.array(line[1]) - np.array(line[0])
    n_v_line = v_line / np.linalg.norm(v_line)
    v_l0_point = np.array(point) - np.array(line[0])

    alpha = clamp(np.dot(v_l0_point, n_v_line)/np.linalg.norm(v_line), 0, 1)

    p_closest = np.array(line[0]) + alpha*v_line
    return p_closest, alpha


def closest_point(path, query_point):
    """
    Computes the closest point on the path to the query point
    
    Returns:
    point, ind, alpha
    point: closest point on path to query point
    ind: index of the preceeding point of the path to point
    alpha: The fraction from path[ind] to path[ind+1] where path is
    """
    d_close = dist(path[0], query_point)
    alpha_close = 0
    point_close = path[0]
    ind_close = 0
    for ind in range(len(path)-1):
        p, alpha = closest_point_to_line([path[ind], path[ind+1]], query_point)
        d = dist(p, query_point)
        if d < d_close:
            d_close = d
            alpha_close = alpha
            point_close = p
            ind_close = ind
    if alpha_close == 1:
        alpha_close = 0
        ind_close += 1
        
    return point_close, ind_close, alpha_close


def travel_along(path, distance, starting_point=None):
    """
    Travels along the path from the starting point for distance

    Parameters:
    path: path
    distance: total euclidean distance to travel. Negative follows backwards
    starting_point: path traversal starts at the closest point on the path to this point

    Returns: 
    new_path: subpath which lies completely on original path while following inputs as best as possible
    """

    direction = np.sign(distance)
    dist_to_go = abs(distance)

    q, ind, alpha = closest_point(path, starting_point)
    newpath = [q]

    path = np.array(path)


    if alpha != 0:
        ind += 1
        path = np.concatenate((path[0:ind], [q], path[ind:]))

    


    while dist_to_go > 0:
        if direction == 1 and ind == len(path) - 1:
            return newpath

        if direction == -1 and ind == 0:
            return newpath

        ind = ind + int(direction)
        dist_to_next = dist(q, path[ind])

        if dist_to_next > dist_to_go:
            motion = path[ind] - q
            motion = motion/np.linalg.norm(motion)
            newpath.append(motion*dist_to_go + q)
            break

        q = path[ind]
        newpath.append(q)
        dist_to_go -= dist_to_next
        

    return newpath



def path_length(path):
    if len(path) == 0:
        return 0

    path = np.array(path)
    q = path[0]
    d = 0
    for ind in range(1,len(path)):
        d += dist(q, path[ind])
        q = path[ind]

    return d
