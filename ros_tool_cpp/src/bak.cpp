#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <iostream>

void readBag(const std::string &bag_path)
{
  // 创建读取器
  rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

  // 打开bag文件
  rosbag2_cpp::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);

  // 读取消息
  while (reader.has_next())
  {
    auto serialized_message = reader.read_next();

    std::cout << "\nTopic: " << serialized_message->topic_name << std::endl;
    std::cout << "Timestamp: " << serialized_message->time_stamp << std::endl;
    std::cout << "Data: " << serialized_message->serialized_data << std::endl;

    // 这里可以添加特定消息类型的反序列化代码
  }

  // 获取所有topic和类型
  auto topics = reader.get_all_topics_and_types();

  std::cout << "Bag文件中的Topics列表:" << std::endl;
  for (const auto &topic : topics)
  {
    std::cout << "- " << topic.name << " [类型: " << topic.type << "]" << std::endl;
  }
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <bag_path>" << std::endl;
    return 1;
  }

  std::string bag_path = argv[1];
  readBag(bag_path);
  return 0;
}