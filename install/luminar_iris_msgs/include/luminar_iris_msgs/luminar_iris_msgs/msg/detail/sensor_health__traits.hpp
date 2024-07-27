// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__TRAITS_HPP_
#define LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "luminar_iris_msgs/msg/detail/sensor_health__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace luminar_iris_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorHealth & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: system_voltage
  {
    out << "system_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.system_voltage, out);
    out << ", ";
  }

  // member: system_temperature
  {
    out << "system_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.system_temperature, out);
    out << ", ";
  }

  // member: system_mode
  {
    out << "system_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.system_mode, out);
    out << ", ";
  }

  // member: system_ok
  {
    out << "system_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.system_ok, out);
    out << ", ";
  }

  // member: laser_ok
  {
    out << "laser_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.laser_ok, out);
    out << ", ";
  }

  // member: scanner_ok
  {
    out << "scanner_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.scanner_ok, out);
    out << ", ";
  }

  // member: receiver_ok
  {
    out << "receiver_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.receiver_ok, out);
    out << ", ";
  }

  // member: datapath_ok
  {
    out << "datapath_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.datapath_ok, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: system_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.system_voltage, out);
    out << "\n";
  }

  // member: system_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.system_temperature, out);
    out << "\n";
  }

  // member: system_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.system_mode, out);
    out << "\n";
  }

  // member: system_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.system_ok, out);
    out << "\n";
  }

  // member: laser_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "laser_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.laser_ok, out);
    out << "\n";
  }

  // member: scanner_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scanner_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.scanner_ok, out);
    out << "\n";
  }

  // member: receiver_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "receiver_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.receiver_ok, out);
    out << "\n";
  }

  // member: datapath_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "datapath_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.datapath_ok, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorHealth & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace luminar_iris_msgs

namespace rosidl_generator_traits
{

[[deprecated("use luminar_iris_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const luminar_iris_msgs::msg::SensorHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  luminar_iris_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use luminar_iris_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const luminar_iris_msgs::msg::SensorHealth & msg)
{
  return luminar_iris_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<luminar_iris_msgs::msg::SensorHealth>()
{
  return "luminar_iris_msgs::msg::SensorHealth";
}

template<>
inline const char * name<luminar_iris_msgs::msg::SensorHealth>()
{
  return "luminar_iris_msgs/msg/SensorHealth";
}

template<>
struct has_fixed_size<luminar_iris_msgs::msg::SensorHealth>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<luminar_iris_msgs::msg::SensorHealth>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<luminar_iris_msgs::msg::SensorHealth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__TRAITS_HPP_
