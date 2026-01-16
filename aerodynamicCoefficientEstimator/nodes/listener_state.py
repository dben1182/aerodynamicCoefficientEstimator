#Works just like the listner is supposed to, but in this case,
#I am just attempting to listen to 
from typing import DefaultDict
from rosplane_msgs.msg import State
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
import pandas as pd

from rosflight_msgs.msg import (
    Airspeed,
    Attitude,
    Barometer,
    Command,
    OutputRaw,
    RCRaw,
    Status,
)


#dictionary where we index with the string like /estimated_state,
#and store the actual class definition like State, Imu, and Command
TOPICS_DESIRED = {
    "/estimated_state": State,
    "/imu/data": Imu,
    "/command": Command
}

#temp dictionary to store just the state
TOPICS_STATE = {
    "/imu/data": Imu,
}

#define sthe read topics function 
def read_topics(bagPath: str,
                validTopics: dict):

    #
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bagPath, storage_id="sqlite3")
    converter_options = ConverterOptions("","")
    reader.open(storage_options, converter_options)

    buckets = DefaultDict(list)
    
    #iterates while the reader has more values
    while reader.has_next():

        #gets the next
        topicName_str, serializedData, timeStamp = reader.read_next()

        #checks if the current topic name string is not one of the 
        #dictionary lists of valid topics from the input
        if topicName_str not in validTopics:
            #if it's not in there, we omit it and weed it out
            continue
        
        #with the topic name, we get the topic assignment, which means
        #the actual topic definition
        msg_type = validTopics[topicName_str]
        
        #gets the current deserialized data
        deserialized_data = deserialize_message(serialized_message=serializedData,
                                                message_type=msg_type) 
        
        #gets the ts time sample
        buckets[topicName_str].append((timeStamp, deserialized_data))
        testPoint = 0
    
    return buckets

#creates the function to convert from messages to dataframes
def msg_to_df(topic, data):


    rows = []


    for ts, msg in data:
        
        if topic == "/imu/data":
            rows.append({
                't': ts,
                'ax': msg.linear_acceleration.x,
                'ay': msg.linear_acceleration.y,
                'az': msg.linear_acceleration.z,
                'gx': msg.angular_acceleration.x,
                'gy': msg.angular_acceleration.y,
                'gz': msg.angular_acceleration.z,
            })
    
    return pd.DataFrame(rows).set_index("t")

def main():
    filePath = Path(__file__).resolve()

    srcPath = filePath.parents[3]

    #gets the path of the bag both as a Path object, and also as a string
    bagPath = srcPath / "anaconda_sys_id" / "data" / "08_28_25_test_1"
    bagPathStr = str(bagPath)
    
    buckets = read_topics(bagPath=bagPathStr,
                          validTopics=TOPICS_STATE) 

    #gets the imu as a dataframe
    df_imu = msg_to_df('/imu/data', buckets['/imu/data'])

    testPoint = 0

if __name__ == "__main__":
    main()
