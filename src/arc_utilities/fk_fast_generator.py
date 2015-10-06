#!/usr/bin/python

#################################################
#                                               #
#   Calder Phillips-Grafflin - WPI/ARC Lab      #
#                                               #
#   Reference Python implementation of the      #
#   eXtensible Trajectory File format.          #
#                                               #
#################################################

import xml.dom.minidom as MD
import xml.etree.ElementTree as ET
import re
import math
import StringIO


def generate_eigen_transform_from_translation_and_rotation(translation, quaternion, transform_name):
    translation_str = "Eigen::Translation3d " + transform_name + "_translation(" + translation.x + ", " + translation.y + ", " + translation.z + ");\n"
    rotation_str = "Eigen::Quaterniond " + transform_name + "_rotation(" + quaternion.w + ", " + quaternion.x + ", " + quaternion.y + ", " + quaternion.z + ");\n"
    transform_str = "Eigen::Affine3d " + transform_name + " = " + transform_name + "_translation * " + transform_name + "_rotation;\n"
    transform_code_gen_str = translation_str + rotation_str + transform_str
    return transform_code_gen_str

def generate_joint_link_transform(origin_to_joint_translation, origin_to_joint_quaternion, joint_axis, joint_link_name):
    assert(joint_axis == 'x' or joint_axis == 'y' or joint_axis == 'z')
