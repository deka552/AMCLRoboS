from rosbags.rosbag2 import Reader
from rosbags.rosbag1 import Writer
with Reader('stand2_0.db3') as reader, Writer('output.bag') as writer:
    for conn, timestamp, rawdata in reader.messages():
        writer.write(conn.topic, rawdata, timestamp)
