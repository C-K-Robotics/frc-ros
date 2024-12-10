def get_what_i_can_handle():
    return "builtin_interfaces/Time"


def get_handle_able_type():
    from builtin_interfaces.msg import Time

    return Time


def handle(msg):
    return {"stamp_sec": msg.sec + msg.nanosec / 1e9}
