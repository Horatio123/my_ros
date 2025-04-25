from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
from collections import OrderedDict
from rclpy_message_converter import message_converter

header = (
    Header(
        stamp=Time(sec=1744552264, nanosec=566748602),
        frame_id="imu_link",
    ),
)

imu_info = Imu(
    header=Header(
        stamp=Time(sec=1744552264, nanosec=566748602),
        frame_id="imu_link",
    ),
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    orientation_covariance=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    angular_velocity=Vector3(x=0.0, y=-0.006, z=-0.015),
    angular_velocity_covariance=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    linear_acceleration=Vector3(x=-1.072, y=-0.5670000000000001, z=-10.475),
    linear_acceleration_covariance=np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ),
)

msg = imu_info.header
for field_name, field_type in zip(msg.__slots__, msg.SLOT_TYPES):
      value = getattr(msg, field_name, None)
      print(f"Field: {field_name}, Type: {field_type}, Value: {value}")


# print(imu_info)
# print(isinstance(header, OrderedDict))
print("type(imu_info.header)", type(imu_info.header))
print("type(imu_info.header.stamp)", type(imu_info.header.stamp))
print("type(imu_info.header.stamp.sec)", type(imu_info.header.stamp.sec))
print("type(imu_info.header.stamp.nanosec)", type(imu_info.header.stamp.nanosec))
print("type(imu_info.header.frame_id): ", type(imu_info.header.frame_id))


header = message_converter.convert_ros_message_to_dictionary(imu_info.header)
print("header: ", header)
# print("type(imu_info.orientation.x)", type(imu_info.orientation.x))
# print("type(imu_info.linear_acceleration_covariance)", type(imu_info.linear_acceleration_covariance))
# print(imu_info.header.__slots__)
# print(imu_info.header.SLOT_TYPES)
