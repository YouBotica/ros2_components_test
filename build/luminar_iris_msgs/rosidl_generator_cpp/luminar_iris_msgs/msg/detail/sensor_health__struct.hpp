// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__STRUCT_HPP_
#define LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__luminar_iris_msgs__msg__SensorHealth __attribute__((deprecated))
#else
# define DEPRECATED__luminar_iris_msgs__msg__SensorHealth __declspec(deprecated)
#endif

namespace luminar_iris_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorHealth_
{
  using Type = SensorHealth_<ContainerAllocator>;

  explicit SensorHealth_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->system_voltage = 0.0f;
      this->system_temperature = 0.0f;
      this->system_mode = 0;
      this->system_ok = false;
      this->laser_ok = false;
      this->scanner_ok = false;
      this->receiver_ok = false;
      this->datapath_ok = false;
    }
  }

  explicit SensorHealth_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->system_voltage = 0.0f;
      this->system_temperature = 0.0f;
      this->system_mode = 0;
      this->system_ok = false;
      this->laser_ok = false;
      this->scanner_ok = false;
      this->receiver_ok = false;
      this->datapath_ok = false;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _system_voltage_type =
    float;
  _system_voltage_type system_voltage;
  using _system_temperature_type =
    float;
  _system_temperature_type system_temperature;
  using _system_mode_type =
    uint8_t;
  _system_mode_type system_mode;
  using _system_ok_type =
    bool;
  _system_ok_type system_ok;
  using _laser_ok_type =
    bool;
  _laser_ok_type laser_ok;
  using _scanner_ok_type =
    bool;
  _scanner_ok_type scanner_ok;
  using _receiver_ok_type =
    bool;
  _receiver_ok_type receiver_ok;
  using _datapath_ok_type =
    bool;
  _datapath_ok_type datapath_ok;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__system_voltage(
    const float & _arg)
  {
    this->system_voltage = _arg;
    return *this;
  }
  Type & set__system_temperature(
    const float & _arg)
  {
    this->system_temperature = _arg;
    return *this;
  }
  Type & set__system_mode(
    const uint8_t & _arg)
  {
    this->system_mode = _arg;
    return *this;
  }
  Type & set__system_ok(
    const bool & _arg)
  {
    this->system_ok = _arg;
    return *this;
  }
  Type & set__laser_ok(
    const bool & _arg)
  {
    this->laser_ok = _arg;
    return *this;
  }
  Type & set__scanner_ok(
    const bool & _arg)
  {
    this->scanner_ok = _arg;
    return *this;
  }
  Type & set__receiver_ok(
    const bool & _arg)
  {
    this->receiver_ok = _arg;
    return *this;
  }
  Type & set__datapath_ok(
    const bool & _arg)
  {
    this->datapath_ok = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SYSTEM_STATUS_UNKNOWN =
    0u;
  static constexpr uint8_t SYSTEM_STATUS_STANDBY =
    1u;
  static constexpr uint8_t SYSTEM_STATUS_ACTIVE =
    2u;
  static constexpr uint8_t SYSTEM_STATUS_SHUTDOWN =
    3u;

  // pointer types
  using RawPtr =
    luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator> *;
  using ConstRawPtr =
    const luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__luminar_iris_msgs__msg__SensorHealth
    std::shared_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__luminar_iris_msgs__msg__SensorHealth
    std::shared_ptr<luminar_iris_msgs::msg::SensorHealth_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorHealth_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->system_voltage != other.system_voltage) {
      return false;
    }
    if (this->system_temperature != other.system_temperature) {
      return false;
    }
    if (this->system_mode != other.system_mode) {
      return false;
    }
    if (this->system_ok != other.system_ok) {
      return false;
    }
    if (this->laser_ok != other.laser_ok) {
      return false;
    }
    if (this->scanner_ok != other.scanner_ok) {
      return false;
    }
    if (this->receiver_ok != other.receiver_ok) {
      return false;
    }
    if (this->datapath_ok != other.datapath_ok) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorHealth_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorHealth_

// alias to use template instance with default allocator
using SensorHealth =
  luminar_iris_msgs::msg::SensorHealth_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorHealth_<ContainerAllocator>::SYSTEM_STATUS_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorHealth_<ContainerAllocator>::SYSTEM_STATUS_STANDBY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorHealth_<ContainerAllocator>::SYSTEM_STATUS_ACTIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorHealth_<ContainerAllocator>::SYSTEM_STATUS_SHUTDOWN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace luminar_iris_msgs

#endif  // LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__STRUCT_HPP_
