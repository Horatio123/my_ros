import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import json
import cv2
import numpy as np
from sensor_msgs.msg import Image
import os


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
    image_count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/camera/color/image_raw":
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        # 将ROS Image消息转换为OpenCV格式
        try:
            if msg.encoding in ["rgb8", "bgr8"]:
                # 创建输出目录（如果不存在）
                output_dir = "output"
                os.makedirs(output_dir, exist_ok=True)

                # 转换图像数据为numpy数组
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, -1
                )

                # 保存为PNG文件
                filename = f"{output_dir}/image_{timestamp}.png"
                cv2.imwrite(filename, cv_image)
                print(f"保存图片成功: {filename}")
                image_count += 1
            else:
                print(f"不支持的图像编码格式: {msg.encoding}")
        except Exception as e:
            print(f"图像处理失败: {str(e)}")
        return

    print(f"共保存 {image_count} 张图片")

    # 关闭读取器
    del reader


if __name__ == "__main__":
    # bag_path = "/home/ros/bags/joy.bag"
    bag_path = "/home/ros/bags/image.bag"
    read_bag(bag_path)

# source /opt/ros/foxy/setup.bash
# source /home/ros/rosbag2/install/setup.bash
# source /home/ros/rospy_message_converter/install/setup.bash

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
