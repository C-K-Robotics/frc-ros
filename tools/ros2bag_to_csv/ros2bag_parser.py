"""
Haoru Xue | haorux@andrew.cmu.edu | August 2021

Data utils
"""

from pathlib import Path
from typing import Union
import multiprocessing
from rclpy.serialization import deserialize_message


class MessageIterator:
    def __init__(self, cur, message_type: str, buffer_size: int = 1000) -> None:
        self.cur = cur
        self.len = self.cur.rowcount
        self.message_type = message_type
        self.buffer_size = buffer_size

    def __iter__(self):
        while True:
            buffer = self.__fetch_some()
            if not buffer:
                break
            for timestamp, message in buffer:
                yield timestamp, message

    def __deserialize(self, row):
        timestamp, data = row
        return timestamp, deserialize_message(data, self.message_type)

    def __fetch_some(self):
        pass

        rows = self.cur.fetchmany(self.buffer_size)

        with multiprocessing.Pool() as p:
            result = p.map(self.__deserialize, rows)
        return result

    def __len__(self):
        return self.len


class SqlBagFileParser:
    """
    https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/?answer=377686#post-id-377686
    """

    def __init__(self, bag_file: Union[str, Path]):
        import sqlite3

        self.conn = sqlite3.connect(str(bag_file))
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: type_of for id_of, name_of, type_of in topics_data}

    def __del__(self):
        if self.conn:
            self.conn.close()

    def get_messages(self, topic_name: str, lazy=False):
        """get converted messages from ROS2 bag

        Args:
            topic_name (str): name of the topic to extract.
            lazy (bool, optional): If true, return a MessageIterator.
                                   If false, return a list of all the messages.
                                   Defaults to False.

        Returns:
            [type]: [description]
        """
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import deserialize_message

        if not topic_name in self.topic_msg_message:
            return None
        topic_id = self.topic_id[topic_name]

        message_type = get_message(self.topic_msg_message[topic_name])
        if not lazy:
            # Get from the db
            rows = self.cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
            ).fetchall()
            # Deserialise all and timestamp them
            return [
                (timestamp, deserialize_message(data, message_type)) for timestamp, data in rows
            ]
        else:
            rows_cursor = self.cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
            )
            return MessageIterator(rows_cursor, message_type)
