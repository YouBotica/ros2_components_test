// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice
#include "luminar_iris_msgs/msg/detail/sensor_health__rosidl_typesupport_fastrtps_cpp.hpp"
#include "luminar_iris_msgs/msg/detail/sensor_health__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Time &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces


namespace luminar_iris_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
cdr_serialize(
  const luminar_iris_msgs::msg::SensorHealth & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.stamp,
    cdr);
  // Member: battery_voltage
  cdr << ros_message.battery_voltage;
  // Member: system_voltage
  cdr << ros_message.system_voltage;
  // Member: system_temperature
  cdr << ros_message.system_temperature;
  // Member: system_mode
  cdr << ros_message.system_mode;
  // Member: system_ok
  cdr << (ros_message.system_ok ? true : false);
  // Member: laser_ok
  cdr << (ros_message.laser_ok ? true : false);
  // Member: scanner_ok
  cdr << (ros_message.scanner_ok ? true : false);
  // Member: receiver_ok
  cdr << (ros_message.receiver_ok ? true : false);
  // Member: datapath_ok
  cdr << (ros_message.datapath_ok ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  luminar_iris_msgs::msg::SensorHealth & ros_message)
{
  // Member: stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.stamp);

  // Member: battery_voltage
  cdr >> ros_message.battery_voltage;

  // Member: system_voltage
  cdr >> ros_message.system_voltage;

  // Member: system_temperature
  cdr >> ros_message.system_temperature;

  // Member: system_mode
  cdr >> ros_message.system_mode;

  // Member: system_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.system_ok = tmp ? true : false;
  }

  // Member: laser_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.laser_ok = tmp ? true : false;
  }

  // Member: scanner_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.scanner_ok = tmp ? true : false;
  }

  // Member: receiver_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.receiver_ok = tmp ? true : false;
  }

  // Member: datapath_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.datapath_ok = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
get_serialized_size(
  const luminar_iris_msgs::msg::SensorHealth & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: stamp

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.stamp, current_alignment);
  // Member: battery_voltage
  {
    size_t item_size = sizeof(ros_message.battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: system_voltage
  {
    size_t item_size = sizeof(ros_message.system_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: system_temperature
  {
    size_t item_size = sizeof(ros_message.system_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: system_mode
  {
    size_t item_size = sizeof(ros_message.system_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: system_ok
  {
    size_t item_size = sizeof(ros_message.system_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: laser_ok
  {
    size_t item_size = sizeof(ros_message.laser_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: scanner_ok
  {
    size_t item_size = sizeof(ros_message.scanner_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: receiver_ok
  {
    size_t item_size = sizeof(ros_message.receiver_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: datapath_ok
  {
    size_t item_size = sizeof(ros_message.datapath_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
max_serialized_size_SensorHealth(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: stamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: system_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: system_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: system_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: system_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: laser_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: scanner_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: receiver_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: datapath_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = luminar_iris_msgs::msg::SensorHealth;
    is_plain =
      (
      offsetof(DataType, datapath_ok) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SensorHealth__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const luminar_iris_msgs::msg::SensorHealth *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SensorHealth__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<luminar_iris_msgs::msg::SensorHealth *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SensorHealth__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const luminar_iris_msgs::msg::SensorHealth *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SensorHealth__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SensorHealth(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SensorHealth__callbacks = {
  "luminar_iris_msgs::msg",
  "SensorHealth",
  _SensorHealth__cdr_serialize,
  _SensorHealth__cdr_deserialize,
  _SensorHealth__get_serialized_size,
  _SensorHealth__max_serialized_size
};

static rosidl_message_type_support_t _SensorHealth__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SensorHealth__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace luminar_iris_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_luminar_iris_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<luminar_iris_msgs::msg::SensorHealth>()
{
  return &luminar_iris_msgs::msg::typesupport_fastrtps_cpp::_SensorHealth__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, luminar_iris_msgs, msg, SensorHealth)() {
  return &luminar_iris_msgs::msg::typesupport_fastrtps_cpp::_SensorHealth__handle;
}

#ifdef __cplusplus
}
#endif
