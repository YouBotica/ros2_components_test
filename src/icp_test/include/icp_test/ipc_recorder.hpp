#ifndef IPC_RECORDER_HPP_
#define IPC_RECORDER_HPP_

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
// #include <mcap/mcap.hpp> 
#include <std_msgs/msg/int32.hpp>

class IPCRecorder : public rclcpp::Node
{
public:
  IPCRecorder(const std::string & name, const std::string & input_topic);

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg);
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriptions_;
};

#endif // IPC_RECORDER_HPP_
