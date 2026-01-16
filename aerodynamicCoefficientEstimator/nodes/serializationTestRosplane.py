from rosplane_msgs.msg import State
from rclpy.serialization import serialize_message, deserialize_message


def main():

    tempMsg = State()
    #sets the data
    tempMsg.p_d = 1.0
    tempMsg.p_e = 2.0
    tempMsg.p_n = 3.0


    tempMsg_serialized = serialize_message(tempMsg)

    tempMsg_deserialized = deserialize_message(serialized_message=tempMsg_serialized,
                                               message_type=State)

    print(tempMsg_deserialized.p_d)

    tempTest = State

    tempPoint = 0


if __name__ == "__main__":
    main()
