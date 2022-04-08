import ctypes
import struct

import numpy as np
import rospy
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2, Image, PointField
import sensor_msgs.point_cloud2


##################################################################################
# Copied and slightly modified from realsense2_camera/scripts/rs2_listener.py

def pointcloud_to_xyzrgb(pointcloud):
    points = np.array(
        [pc2_to_xyzrgb(pp) for pp in
         sensor_msgs.point_cloud2.read_points(pointcloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))])
    return points


def pc2_to_xyzrgb(point):
    # Thanks to Panos for his code used in this function.
    x, y, z = point[:3]
    rgb = point[3]

    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', rgb)
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
    return x, y, z, r / 255.0, g / 255.0, b / 255.0


##################################################################################
# Copied and slightly modified from https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb


def xyzrgb_to_pc2(pointcloud, frame_id):
    points = []
    for point in pointcloud:
        x = point[0]
        y = point[1]
        z = point[2]
        r = int(point[3] * 255.0)
        g = int(point[4] * 255.0)
        b = int(point[5] * 255.0)
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        points.append([x, y, z, rgb])

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgba', 12, PointField.UINT32, 1),
    ]

    header = Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()

    pc2 = sensor_msgs.point_cloud2.create_cloud(header, fields, points)
    return pc2

##################################################################################
