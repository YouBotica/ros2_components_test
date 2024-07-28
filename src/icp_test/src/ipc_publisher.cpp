#include "icp_test/ipc_publisher.hpp"

Producer::Producer(const std::string & name, const std::string & output)
: Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // Create a publisher on the output topic.
  pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
  std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
  // Create a timer which publishes on the output topic at ~1Hz.
  auto callback = [captured_pub]() -> void {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return;
      }
      static int32_t count = 0;
      std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
      msg->data = count++;
      printf(
        "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };
  timer_ = this->create_wall_timer(1s, callback);
}
