import rclpy
import pandas as pd
from pathlib import Path

from rclpy.node import Node
import rosbag2_py
from std_msgs.msg import String
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from rosflight_msgs.msg import (
    Airspeed,
    Attitude,
    Barometer,
    Command,
    OutputRaw,
    RCRaw,
    Status,
)

from rosplane_msgs.msg import State, CurrentPath, Waypoint
from sensor_msgs.msg import Imu, BatteryState, NavSatFix, Temperature, MagneticField

TOPIC_TYPES = {
    "/airspeed": Airspeed,
    "/attitude": Attitude,
    "/baro": Barometer,
    "/battery": BatteryState,
    "/command": Command,
    "/controller_command": Command,
    "/estimated_state": State,
    "/gnss": NavSatFix,
    "/imu/data": Imu,
    "/imu/temperature": Temperature,
    "/magnetometer": MagneticField,
    "/output_raw": OutputRaw,
    "/rc_raw": RCRaw,
    "/status": Status,
}

#defines the function to read all the topics in    
def read_topics(bag_path, topics):
   
    reader = SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("","")
    reader.open(storage_options, converter_options)


    topics_info = reader.get_all_topics_and_types()
    topicsNames = []
    topicsTypes = []
    for currentTopic in topics_info:

        print(currentTopic.name, currentTopic.type)
        topicsNames.append(currentTopic.name)
        topicsTypes.append(currentTopic.type)

    buckets = defaultdict(list)
    
    while reader.has_next():
        
        #reads the next topic
        topic, raw, t = reader.read_next()

        if topic not in topics:
            continue
    
    potato = 0
    '''
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions("cdr", "cdr")
    )

    buckets = defaultdict(list)

    while reader.has_next():

        topic, raw, t = reader.read_next()

        if topic not in topics:
            continue

        msg_type = topics[topic]
        msg = deserialize_message(serialized_message=raw,
                                  message_type=msg_type)

        ts = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        buckets[topic].append((ts, msg))
    return buckets

    #'''

def msg_to_df(topic, data):
    rows = []

    for ts, msg in data:

        if topic == "/imu/data":
            rows.append({
                "t": ts,
                "ax": msg.linear_acceleration.x,
                "ay": msg.linear_acceleration.y,
                "az": msg.linear_acceleration.z,
                "gx": msg.angular_velocity.x,
                "gy": msg.angular_velocity.y,
                "gz": msg.angular_velocity.z,
            })

        elif topic == "/estimated_state":
            rows.append({
                "t": ts,
                "pn": msg.position[0],
                "pe": msg.position[1],
                "pd": msg.position[2],
                "vn": msg.Va,
                "ve": msg.Vg,
                "vd": msg.chi,
            })

        elif topic == "/command":
            rows.append({
                "t": ts,
                "throttle": msg.throttle,
                "aileron": msg.x,
                "elevator": msg.y,
                "rudder": msg.F,
            })

    return pd.DataFrame(rows).set_index("t")


def synchronize(df_state, df_imu, df_cmd, tol=0.01):
    return (
        df_state
        .join(df_imu, how="nearest", rsuffix="_imu", tolerance=tol)
        .join(df_cmd, how="nearest", rsuffix="_cmd", tolerance=tol)
    )

TOPICS = {
    "/estimated_state": State,
    "/imu/data": Imu,
    "/command": Command
}


def main():


    p0 = Path(__file__).resolve()
    p0_str = str(p0)

    p1 = p0.parents[3]

    bagPath = p1 / "anaconda_sys_id" / "data" / "08_28_25_test_1"
    bagPathStr = str(bagPath)

    
    #'''
    
    #creates the buckets
    buckets = read_topics(bag_path=bagPathStr, topics=TOPICS)

    df_state = msg_to_df("/estimated_state", buckets["/estimated_state"])
    df_imu = msg_to_df("/imu/data", buckets["/imu/data"])
    df_cmd = msg_to_df("/command", buckets["/command"])

    df_sync = synchronize(df_state=df_state, 
                          df_imu=df_imu,
                          df_cmd=df_cmd)


    print(df_sync.head())

    
    #sets the path of the rosbag
    tomato = 0

    potato = 0
    #'''

if __name__ == "__main__":
    main()
