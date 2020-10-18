import math

from arc_utilities.color_mapping import interpolate_hot_to_cold
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


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
