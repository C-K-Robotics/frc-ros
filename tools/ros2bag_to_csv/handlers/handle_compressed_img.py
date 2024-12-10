def get_what_i_can_handle():
    return "sensor_msgs/CompressedImage"


def get_handle_able_type():
    from sensor_msgs.msg import CompressedImage

    return CompressedImage


def handle(msg):
    return {}