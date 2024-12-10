from . import handle_pose


def get_what_i_can_handle():
    return "geometry_msgs/PoseWithCovariance"


def get_handle_able_type():
    from geometry_msgs.msg import PoseWithCovariance

    return PoseWithCovariance


def handle(msg):
    return handle_pose.handle(msg.pose)
