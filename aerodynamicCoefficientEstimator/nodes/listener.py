import rclpy
import pandas as pd
from pathlib import Path
from rclpy.node import Node
import rosbag2_py
from std_msgs.msg import String
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosplane_msgs.msg import State, CurrentPath, Waypoint
from sensor_msgs.msg import Imu, BatteryState, NavSatFix, Temperature, MagneticField

#gets a large list of rosflight messages
from rosflight_msgs.msg import (
    Airspeed,
    Attitude,
    Barometer,
    Command,
    OutputRaw,
    RCRaw,
    Status,
)

#gets a large list of topic types
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


TOPICS_DESIRED = {
    "/estimated_state": State,
    "/imu/data": Imu,
    "/command": Command
}

#defines the function to read the topics
#Arguments:
#bagPath: the string absolute path to the rosbag
#validTopics: the dictionary of valid topics for us to actually read and understand
def read_topics(bagPath: str,
                validTopics: dict):
    
    #creates the sequential reader
    reader = SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bagPath, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("","")
    reader.open(storage_options, converter_options)

    #creates the buckets
    buckets = defaultdict(list)
    
    #section to get all of the topics and types
    topics_metadata = reader.get_all_topics_and_types()
    allTopicsNames = []
    allTopicsTypes = []
    for currentMetadata in topics_metadata:
        print(currentMetadata.name, currentMetadata.type)
        allTopicsNames.append(currentMetadata.name)
        allTopicsTypes.append(currentMetadata.type)

        testPoint = 0

    #iterates over while the reader has more
    while reader.has_next():

        topic, raw, t = reader.read_next()
        
        #filters out the undesirable topics
        if topic not in validTopics:
            continue

        #gets the message type
        msg_type = validTopics[topic]

        testPoint = 0

    testPoint = 0


def main():


    #gets the current file path
    p0 = Path(__file__).resolve()
    p0_str = str(p0)

    p1 = p0.parents[3]


    bagPath = p1 / "anaconda_sys_id" / "data" / "08_28_25_test_1"
    bagPathStr = str(bagPath)

    #calls the read topics function
    read_topics(bagPath=bagPathStr,
                validTopics=TOPICS_DESIRED)

    testPoint = 0

if __name__ == "__main__":
    main()
