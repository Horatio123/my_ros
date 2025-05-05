#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



void laserscan_to_pcd(const std::string &bag_path,
                      const std::string &topic,
                      const std::string &output_pcd_path)
{
  rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  rosbag2_cpp::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  reader.open(storage_options, converter_options);

  auto topics = reader.get_all_topics_and_types();

  laser_geometry::LaserProjection projector; // 核心转换器
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
  auto pointcloud_num = 0;

  while (reader.has_next())
  {
    auto bag_message = reader.read_next();
    if (bag_message->topic_name == topic)
    {
      sensor_msgs::msg::LaserScan scan_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(&extracted_serialized_msg, &scan_msg);

      // 使用laser_geometry进行转换
      sensor_msgs::msg::PointCloud2 cloud_msg;
      projector.projectLaser(scan_msg, cloud_msg);

      // 保存为PCD文件
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(cloud_msg, pcl_cloud);
      std::string output_pcd_object = output_pcd_path + std::to_string(pointcloud_num) + ".pcd";
      if (pcl::io::savePCDFileASCII(output_pcd_object, pcl_cloud) == -1)
      {
          std::cerr << "保存PCD文件失败" << std::endl;
      }
      else
      {
          std::cout << "成功保存点云到 " << output_pcd_object << std::endl;
          pointcloud_num++;
      }
    }
  }
}
int main(int argc, char **argv)
{
  std::string bag_path = "/home/ros/bags/data2.bag";
  std::string topic = "/scan";
  std::string output_pcd_path = "/home/ros/my_ros/ros_tool_cpp/output/";
  laserscan_to_pcd(bag_path, topic, output_pcd_path);
  return 0;
}