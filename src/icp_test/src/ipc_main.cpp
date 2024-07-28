
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "icp_test/ipc_publisher.hpp"
#include "icp_test/ipc_subscriber.hpp"
#include "icp_test/ipc_recorder.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("producer", "number");
  auto consumer = std::make_shared<Consumer>("consumer", "number");
  // std::vector<std::string> topics {"number"};
  auto recorder = std::make_shared<IPCRecorder>("recorder", "number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.add_node(recorder);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}