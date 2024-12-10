def get_what_i_can_handle():
    return "geometry_msgs/Vector3"


def get_handle_able_type():
    from geometry_msgs.msg import Vector3

    return Vector3


def handle(msg):
    return {"x": msg.x, "y": msg.y, "z": msg.z}
