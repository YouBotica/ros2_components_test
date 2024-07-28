#ifndef CONSUMER_HPP_
#define CONSUMER_HPP_

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class Consumer : public rclcpp::Node
{
public:
  Consumer(const std::string & name, const std::string & input);

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

#endif // CONSUMER_HPP_
