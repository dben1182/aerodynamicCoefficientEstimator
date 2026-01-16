from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rosflight_msgs.msg import Command
from rosplane_msgs.msg import State
from sensor_msgs.msg import Imu

TOPICS_DESIRED = {
    "/estimated_state": State,
    "/imu/data": Imu,
    "/command": Command
}

TestDict = {}

TestDict["/estimated_state"] = State
TestDict["/imu/data"] = Imu
TestDict["/command"] = Command

potato = 0
