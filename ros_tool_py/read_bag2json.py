import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy_message_converter import message_converter
from rclpy_message_converter import json_message_converter
import json


def read_bag(bag_path):
    # 创建读取器
    reader = rosbag2_py.SequentialReader()

    # 打开bag文件
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    # 获取所有主题和类型
    topics = reader.get_all_topics_and_types()
    print(f"topics: {topics}")

    # 创建类型字典
    type_map = {topic.name: topic.type for topic in topics}
    print(f"type_map: {type_map}")

    # 读取所有消息
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/camera/color/image_raw":
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        print(f"\nTopic: {topic}")
        print(f"Timestamp: {timestamp}")
        print(f"Data: {data}")
        print(f"Message: {msg}")
        message_dict = message_converter.convert_ros_message_to_dictionary(msg)
        print(f"message_dict: {message_dict}")
        message_dict_json = json.dumps(message_dict, indent=4)
        print(f"message_dict_json: {message_dict_json}")
        json_str = json_message_converter.convert_ros_message_to_json(msg)
        print(f"json_str: {json_str}")

        with open(f"output.json", "a") as f:
            json.dump(obj=message_dict, fp=f, indent=4)
        return

    # 关闭读取器
    del reader


def read_bag_topic(bag_path):
    # 创建读取器
    reader = rosbag2_py.SequentialReader()

    # 打开bag文件
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)

    # 获取所有主题和类型
    topics = reader.get_all_topics_and_types()
    print(f"topics: {topics}")

    # 创建类型字典
    type_map = {topic.name: topic.type for topic in topics}
    print(f"type_map: {type_map}")

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=["/imu/data_raw"])
    reader.set_filter(storage_filter)

    # 读取所有消息
    imu_info = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        print(f"\nTopic: {topic}")
        print(f"Timestamp: {timestamp}")
        print(f"Data: {data}")
        print(f"Message: {msg}")
        message_dict = message_converter.convert_ros_message_to_dictionary(msg)
        print(f"message_dict: {message_dict}")
        message_dict_json = json.dumps(message_dict, indent=4)
        print(f"message_dict_json: {message_dict_json}")
        json_str = json_message_converter.convert_ros_message_to_json(msg)
        print(f"json_str: {json_str}")
        
        imu_info.append(message_dict)

    imu_info_len = len(imu_info)
    print(f"imu_info_len: {imu_info_len}")
    with open(f"output.json", "w") as f:
        json.dump(obj=imu_info, fp=f, indent=4)
        # return

    # 关闭读取器
    del reader


if __name__ == "__main__":
    bag_path = "/home/ros/bags/joy.bag"
    # read_bag(bag_path)
    read_bag_topic(bag_path)

# - /joy/set_feedback [类型: sensor_msgs/msg/JoyFeedback]
# - /voltage [类型: std_msgs/msg/Float32]
# - /rosout [类型: rcl_interfaces/msg/Log]
# - /vel_raw [类型: geometry_msgs/msg/Twist]
# - /move_base/cancel [类型: actionlib_msgs/msg/GoalID]
# - /joy [类型: sensor_msgs/msg/Joy]
# - /parameter_events [类型: rcl_interfaces/msg/ParameterEvent]
# - /cmd_vel [类型: geometry_msgs/msg/Twist]
# - /imu/data_raw [类型: sensor_msgs/msg/Imu]
# - /joint_states [类型: sensor_msgs/msg/JointState]
# - /JoyState [类型: std_msgs/msg/Bool]
# - /Buzzer [类型: std_msgs/msg/Bool]
# - /imu/mag [类型: sensor_msgs/msg/MagneticField]
# - /RGBLight [类型: std_msgs/msg/Int32]
# - /edition [类型: std_msgs/msg/Float32]
