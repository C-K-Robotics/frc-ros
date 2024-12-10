from . import handle_quaternion
from . import handle_vector3


def get_what_i_can_handle():
    return "geometry_msgs/Pose"


def get_handle_able_type():
    from geometry_msgs.msg import Pose

    return Pose


def handle(msg):
    pos = handle_vector3.handle(msg.position)
    ori = handle_quaternion.handle(msg.orientation)

    return {
        "position/x": pos["x"],
        "position/y": pos["y"],
        "position/z": pos["z"],
        "orientation/roll": ori["roll"],
        "orientation/yaw": ori["yaw"],
        "orientation/pitch": ori["pitch"],
    }
