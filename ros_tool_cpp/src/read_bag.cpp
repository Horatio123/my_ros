#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
// Add the missing header
#include <sstream> 

using json = nlohmann::json;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <bag_path> <output_json>" << std::endl;
        return 1;
    }

    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_cpp::StorageOptions storage_options{argv[1], "sqlite3"};
    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
    reader.open(storage_options, converter_options);

    json bag_json;

    auto topics = reader.get_all_topics_and_types();
    json topics_json = json::array();
    for (const auto& topic : topics) {
        topics_json.push_back({
            {"name", topic.name},
            {"type", topic.type},
            {"serialization_format", topic.serialization_format}
        });
    }
    bag_json["topics"] = topics_json;

    bag_json["messages"] = json::array();

    rclcpp::Serialization<geometry_msgs::msg::Twist> serialization;
    while (reader.has_next()) {
        auto msg = reader.read_next();
        if (msg->topic_name == "/cmd_vel") {
            geometry_msgs::msg::Twist ros_msg;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, &ros_msg);

            std::stringstream ss;
            ss << "linear: " << ros_msg.linear.x << ", " << ros_msg.linear.y << ", " << ros_msg.linear.z
               << " angular: " << ros_msg.angular.x << ", " << ros_msg.angular.y << ", " << ros_msg.angular.z;
            std::string message_content = ss.str();

            std::cout << "Message content: " << message_content << std::endl;
            bag_json["messages"].push_back({
                {"topic", msg->topic_name},
                {"timestamp", msg->time_stamp},
                {"content", message_content}
            });
        }
    }

    std::ofstream out(argv[2]);
    out << bag_json.dump(4);
    out.close();

    return 0;
}