from pathlib import Path
import argparse
import csv
from ros2bag_parser import *
import handlers
import datetime
import numpy as np

TYPES_TO_STORE = [int, float, str, bool]

TYPES_TO_IGNORE = [tuple, list, np.array, np.ndarray]


def __update_sub_msg(parent_msg: dict, field_name: str, child_msg: dict):
    for sub_field in child_msg.keys():
        parent_msg[field_name + "/" + sub_field] = child_msg[sub_field]


def __get_local_time(epoch_time_nano) -> str:
    return str(datetime.datetime.fromtimestamp(epoch_time_nano / 1e9))


def parse_bag(input_bag, topic, output_csv):
    sql_bag_paths = []
    for path in Path(input_bag).rglob("*.db3"):
        sql_bag_paths.append(path.absolute())
    n_sql_bag_paths = len(sql_bag_paths)
    
    mcap_bag_paths = []
    for path in Path(input_bag).rglob("*.mcap"):
        mcap_bag_paths.append(path.absolute())
    n_mcap_bag_paths = len(mcap_bag_paths)

    bag_type = ""
    if n_sql_bag_paths == 0 and n_mcap_bag_paths == 0:
        print("No files to convert")
        exit()
    
    if n_mcap_bag_paths + n_sql_bag_paths != max(n_mcap_bag_paths, n_sql_bag_paths):
        print("There seems to be a mix of MCAP and SQL files here. Exit")
        exit()
    
    if n_mcap_bag_paths == max(n_mcap_bag_paths, n_sql_bag_paths):
        bag_type = "mcap"
    elif n_sql_bag_paths == max(n_mcap_bag_paths, n_sql_bag_paths):
        bag_type = "sql"

    with open(output_csv, "w") as output_csv:
        csv_writer = None
        first_row = True
        messages = get_messages(topic, sql_bag_paths, mcap_bag_paths)
        count = 0
        print("Parsing messages")
        for msg in messages:
            msg_dict = {}
            message = None
            if bag_type == "sql":
                message = msg[1]
            elif bag_type == "mcap":
                message = msg.ros_msg
            __recursive_parse(None, message, msg_dict)
            if first_row:
                header = list(msg_dict.keys())
                header.insert(0, "timestamp")
                header.insert(1, "local_time")
                csv_writer = csv.DictWriter(output_csv, header)
                csv_writer.writeheader()
                first_row = False
            if bag_type == "sql":
                msg_dict["timestamp"] = msg[0]
            elif bag_type == "mcap":
                msg_dict["timestamp"] = msg.log_time.timestamp() * 1e9
            msg_dict["local_time"] = __get_local_time(msg_dict["timestamp"])
            csv_writer.writerow(msg_dict)
            del msg, msg_dict
            count += 1
            if count % 100 == 0:
                print(f"\rParsed {count} messages...", end="")

def get_messages(topic, sql_bag_paths, mcap_bag_paths):
    messages = None
    if len(sql_bag_paths) > 0:
        sql_bag_parser = SqlBagFileParser(sql_bag_paths[0])
        messages = sql_bag_parser.get_messages(topic)
    elif len(mcap_bag_paths) > 0:
        from mcap_ros2.reader import read_ros2_messages
        messages = read_ros2_messages(source=mcap_bag_paths[0], topics=[topic])
    return messages

def __recursive_parse(parent_attr: str, msg, output_dict: dict):
    for attr_name in dir(msg):
        field_name = __get_field_name(attr_name, parent_attr)
        attr_val = getattr(msg, attr_name)
        attr_type = type(attr_val)
        if attr_name.startswith("_") or callable(attr_val) or attr_type in TYPES_TO_IGNORE:
            continue
        if attr_type in TYPES_TO_STORE:
            output_dict[field_name] = attr_val
        elif handlers.MSG_HANDLER_MAP.get(attr_type) is not None:
            custom_handler = handlers.MSG_HANDLER_MAP.get(attr_type)
            __update_sub_msg(output_dict, field_name, custom_handler(attr_val))
        elif hasattr(attr_val, "_type") and handlers.MSG_NAME_HANDLER_MAP.get(attr_val._type) is not None:
            custom_handler = handlers.MSG_NAME_HANDLER_MAP.get(attr_val._type)
            __update_sub_msg(output_dict, field_name, custom_handler(attr_val))
        else:
            __recursive_parse(field_name, attr_val, output_dict)


def __get_field_name(child_attr, parent_attr=None):
    if parent_attr is not None:
        return parent_attr + "/" + child_attr
    else:
        return child_attr


def main():

    parser = argparse.ArgumentParser(description="Process some integers.")
    parser.add_argument("-i", "--input", help="Input bag folder", required=True)
    parser.add_argument("-t", "--topic", help="Topic to extract", required=True)
    parser.add_argument("-o", "--output", help="Output CSV file")

    args = parser.parse_args()
    output = args.output
    if output is None:
        output = str(args.topic).replace("/", "__").lstrip("__") + ".csv"
    parse_bag(args.input, args.topic, output)


if __name__ == "__main__":
    main()
