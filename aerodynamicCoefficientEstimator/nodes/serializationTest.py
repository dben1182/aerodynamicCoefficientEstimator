from std_msgs.msg import String 
from rclpy.serialization import serialize_message, deserialize_message

def main():

    tempMsg = String()
    tempMsg.data = "Hello world"
    tempMsg_serialized = serialize_message(tempMsg)
    print("serialized: ", tempMsg_serialized)

    tempMsg_deserialized = deserialize_message(serialized_message=tempMsg_serialized,
                                               message_type=String)

    print(tempMsg_deserialized.data)


    tempPoint = 0

if __name__ == "__main__":


    main()
