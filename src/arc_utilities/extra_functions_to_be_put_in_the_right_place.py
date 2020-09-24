import math

from arc_utilities.color_mapping import interpolate_hot_to_cold
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import ColorRGBA


def make_pose(position, rotation):
    px, py, pz = position
    rx, ry, rz, rw = rotation
    new_pose = Pose()
    new_pose.position.x = px
    new_pose.position.y = py
    new_pose.position.z = pz
    new_pose.orientation.x = rx
    new_pose.orientation.y = ry
    new_pose.orientation.z = rz
    new_pose.orientation.w = rw
    return new_pose


def make_vector(x, y, z):
    new_vector = Vector3()
    new_vector.x = x
    new_vector.y = y
    new_vector.z = z
    return new_vector


def make_point(x, y, z):
    new_point = Point()
    new_point.x = x
    new_point.y = y
    new_point.z = z
    return new_point


def make_unit_point(x, y, z):
    mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    assert (mag > 0.0)
    unit_point = Point()
    unit_point.x = x / mag
    unit_point.y = y / mag
    unit_point.z = z / mag
    return unit_point


def normalize_point(raw_point):
    return make_unit_point(raw_point.x, raw_point.y, raw_point.z)


def safe_color_val(val):
    if val >= 1.0:
        return 1.0
    elif val <= 0.0:
        return 0.0
    else:
        return val


def make_color(r, g, b, a):
    new_color = ColorRGBA()
    new_color.r = safe_color_val(r)
    new_color.g = safe_color_val(g)
    new_color.b = safe_color_val(b)
    new_color.a = safe_color_val(a)
    return new_color


def map_color(value):
    # [r, g, b] = interpolate_jet(value)
    [r, g, b] = interpolate_hot_to_cold(value)
    return make_color(r, g, b, 1.0)
