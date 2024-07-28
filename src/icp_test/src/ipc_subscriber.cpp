#include "icp_test/ipc_subscriber.hpp"

Consumer::Consumer(const std::string & name, const std::string & input)
: Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // Create a subscription on the input topic which prints on receipt of new messages.
  sub_ = this->create_subscription<std_msgs::msg::Int32>(
    input,
    10,
    [](std_msgs::msg::Int32::UniquePtr msg) {
      printf(
        "Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
    });
}
