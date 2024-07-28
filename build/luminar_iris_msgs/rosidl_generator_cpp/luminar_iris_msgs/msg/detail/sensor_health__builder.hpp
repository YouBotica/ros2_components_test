// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__BUILDER_HPP_
#define LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "luminar_iris_msgs/msg/detail/sensor_health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace luminar_iris_msgs
{

namespace msg
{

namespace builder
{

class Init_SensorHealth_datapath_ok
{
public:
  explicit Init_SensorHealth_datapath_ok(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  ::luminar_iris_msgs::msg::SensorHealth datapath_ok(::luminar_iris_msgs::msg::SensorHealth::_datapath_ok_type arg)
  {
    msg_.datapath_ok = std::move(arg);
    return std::move(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_receiver_ok
{
public:
  explicit Init_SensorHealth_receiver_ok(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_datapath_ok receiver_ok(::luminar_iris_msgs::msg::SensorHealth::_receiver_ok_type arg)
  {
    msg_.receiver_ok = std::move(arg);
    return Init_SensorHealth_datapath_ok(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_scanner_ok
{
public:
  explicit Init_SensorHealth_scanner_ok(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_receiver_ok scanner_ok(::luminar_iris_msgs::msg::SensorHealth::_scanner_ok_type arg)
  {
    msg_.scanner_ok = std::move(arg);
    return Init_SensorHealth_receiver_ok(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_laser_ok
{
public:
  explicit Init_SensorHealth_laser_ok(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_scanner_ok laser_ok(::luminar_iris_msgs::msg::SensorHealth::_laser_ok_type arg)
  {
    msg_.laser_ok = std::move(arg);
    return Init_SensorHealth_scanner_ok(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_system_ok
{
public:
  explicit Init_SensorHealth_system_ok(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_laser_ok system_ok(::luminar_iris_msgs::msg::SensorHealth::_system_ok_type arg)
  {
    msg_.system_ok = std::move(arg);
    return Init_SensorHealth_laser_ok(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_system_mode
{
public:
  explicit Init_SensorHealth_system_mode(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_system_ok system_mode(::luminar_iris_msgs::msg::SensorHealth::_system_mode_type arg)
  {
    msg_.system_mode = std::move(arg);
    return Init_SensorHealth_system_ok(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_system_temperature
{
public:
  explicit Init_SensorHealth_system_temperature(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_system_mode system_temperature(::luminar_iris_msgs::msg::SensorHealth::_system_temperature_type arg)
  {
    msg_.system_temperature = std::move(arg);
    return Init_SensorHealth_system_mode(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_system_voltage
{
public:
  explicit Init_SensorHealth_system_voltage(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_system_temperature system_voltage(::luminar_iris_msgs::msg::SensorHealth::_system_voltage_type arg)
  {
    msg_.system_voltage = std::move(arg);
    return Init_SensorHealth_system_temperature(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_battery_voltage
{
public:
  explicit Init_SensorHealth_battery_voltage(::luminar_iris_msgs::msg::SensorHealth & msg)
  : msg_(msg)
  {}
  Init_SensorHealth_system_voltage battery_voltage(::luminar_iris_msgs::msg::SensorHealth::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_SensorHealth_system_voltage(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

class Init_SensorHealth_stamp
{
public:
  Init_SensorHealth_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorHealth_battery_voltage stamp(::luminar_iris_msgs::msg::SensorHealth::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_SensorHealth_battery_voltage(msg_);
  }

private:
  ::luminar_iris_msgs::msg::SensorHealth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::luminar_iris_msgs::msg::SensorHealth>()
{
  return luminar_iris_msgs::msg::builder::Init_SensorHealth_stamp();
}

}  // namespace luminar_iris_msgs

#endif  // LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__BUILDER_HPP_
