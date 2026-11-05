"""Small rosbag2 (sqlite3) reader helpers for offline analysis."""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag, topics=None, every=None):
    """Yield (topic, t_sec, msg). `every` = {topic: n} keeps every n-th message of that topic."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    classes = {}
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if every and topic in every:
            counts[topic] = counts.get(topic, 0) + 1
            if (counts[topic] - 1) % every[topic]:
                continue
        cls = classes.setdefault(topic, get_message(types[topic]))
        yield topic, t * 1e-9, deserialize_message(data, cls)


def image_bgr(msg):
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == 'rgb8':
        return arr[:, :, ::-1].copy()
    return arr.copy()


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
