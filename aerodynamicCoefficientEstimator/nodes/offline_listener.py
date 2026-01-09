from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
import rclpy
from pathlib import Path


# gets the path to this file
here = Path(__file__).resolve()
target = here.parent.parent
