#ifndef PRODUCER_HPP_
#define PRODUCER_HPP_

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class Producer : public rclcpp::Node
{
public:
  Producer(const std::string & name, const std::string & output);

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // PRODUCER_HPP_