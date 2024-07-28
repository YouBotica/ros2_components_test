#include "icp_test/ipc_recorder.hpp"


IPCRecorder::IPCRecorder(const std::string & name, const std::string & input_topic)
: Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  writer_ = std::make_shared<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "rosbag2_data";
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  
  writer_->open(storage_options, converter_options); 

//   writer_->open("rosbag2_data", {"sqlite3", "rosbag2_storage_default_plugins"});

//   for (const auto & topic : input_topics)
//   {
    auto subscription = this->create_subscription<std_msgs::msg::Int32>(
        input_topic, 10,
        [this, input_topic](std_msgs::msg::Int32::SharedPtr msg) { 

            printf(
            "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
            reinterpret_cast<std::uintptr_t>(msg.get()));
            
            rclcpp::SerializedMessage serialized_msg;
            rclcpp::Serialization<std_msgs::msg::Int32> serializer;
            serializer.serialize_message(msg.get(), &serialized_msg);
            writer_->write(serialized_msg, "std_msgs/msg/Int32", input_topic, this->now());
        }
    );
    // subscriptions_.push_back(subscription);
//   }
}
