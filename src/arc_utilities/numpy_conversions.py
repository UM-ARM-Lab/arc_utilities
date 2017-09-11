import numpy as np
from geometry_msgs.msg import *

def ListPointsToNpArray(points, transform = np.eye(4)):
    arr = np.empty(shape=(3, len(points)))
    for ind in range(len(points)):
        point = transform.dot([points[ind].x, points[ind].y, points[ind].z, 1])
        arr[:, ind] = point[0:3]
    return arr