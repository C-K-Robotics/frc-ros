import rosbag2_py
import rclpy
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message, serialize_message

def get_rosbag_options_write(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='mcap')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def get_rosbag_options_read(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='mcap')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def create_topic(writer, topic_name, topic_type, serialization_format='cdr'):
    topic_name = topic_name
    topic = rosbag2_py.TopicMetadata(name=topic_name, type=topic_type,
                                     serialization_format=serialization_format)
    writer.create_topic(topic)

def filter(bag_path_input,bag_path_output, topics):

    storage_options, converter_options = get_rosbag_options_read(bag_path_input)
    storage_options2, converter_options2 = get_rosbag_options_write(bag_path_output)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options2,converter_options2)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    for i in range(len(topic_types)):
        if topic_types[i].name in topics:
            create_topic(writer, topic_types[i].name, topic_types[i].type)

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        # print(topic)
        if topic in topics:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            writer.write(topic,serialize_message(msg),t)

filter("rosbag2_2023_01_04-21_41_38",
        "tum_dry_run_jan_4_2023_filtered",
        [
            '/novatel_bottom/bestgnsspos',
            '/novatel_bottom/bestgnssvel',
            '/novatel_bottom/rawimu',
            '/novatel_bottom/heading2',
            '/novatel_top/bestgnsspos',
            '/novatel_top/bestgnssvel',
            '/novatel_top/rawimu',
            '/novatel_top/heading2',
            '/luminar_front_points',
            '/luminar_left_points',
            '/luminar_right_points',
            '/radar_front/esr_track',
            '/vimba_front_left/image',
            '/vimba_front_left_center/image',
            '/vimba_front_right_center/image',
            '/vimba_front_right/image',
            '/vimba_rear_left/image',
            '/vimba_rear_right/image',
        ])
