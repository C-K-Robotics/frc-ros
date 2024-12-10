import numpy as np


def get_what_i_can_handle():
    return "geometry_msgs/Quaternion"


def get_handle_able_type():
    from geometry_msgs.msg import Quaternion

    return Quaternion


def handle(msg):
    roll, pitch, yaw = euler_from_quaternion(msg)
    return {"roll": roll * 180 / np.pi, "yaw": yaw * 180 / np.pi, "pitch": pitch * 180 / np.pi}


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
