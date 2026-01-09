import rclpy

from rclpy.node import Node
from std_msgs.msg import String

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
