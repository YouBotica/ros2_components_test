// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice
#include "luminar_iris_msgs/msg/detail/sensor_health__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "luminar_iris_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "luminar_iris_msgs/msg/detail/sensor_health__struct.h"
#include "luminar_iris_msgs/msg/detail/sensor_health__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "builtin_interfaces/msg/detail/time__functions.h"  // stamp

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_luminar_iris_msgs
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_luminar_iris_msgs
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_luminar_iris_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();


using _SensorHealth__ros_msg_type = luminar_iris_msgs__msg__SensorHealth;

static bool _SensorHealth__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SensorHealth__ros_msg_type * ros_message = static_cast<const _SensorHealth__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->stamp, cdr))
    {
      return false;
    }
  }

  // Field name: battery_voltage
  {
    cdr << ros_message->battery_voltage;
  }

  // Field name: system_voltage
  {
    cdr << ros_message->system_voltage;
  }

  // Field name: system_temperature
  {
    cdr << ros_message->system_temperature;
  }

  // Field name: system_mode
  {
    cdr << ros_message->system_mode;
  }

  // Field name: system_ok
  {
    cdr << (ros_message->system_ok ? true : false);
  }

  // Field name: laser_ok
  {
    cdr << (ros_message->laser_ok ? true : false);
  }

  // Field name: scanner_ok
  {
    cdr << (ros_message->scanner_ok ? true : false);
  }

  // Field name: receiver_ok
  {
    cdr << (ros_message->receiver_ok ? true : false);
  }

  // Field name: datapath_ok
  {
    cdr << (ros_message->datapath_ok ? true : false);
  }

  return true;
}

static bool _SensorHealth__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SensorHealth__ros_msg_type * ros_message = static_cast<_SensorHealth__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->stamp))
    {
      return false;
    }
  }

  // Field name: battery_voltage
  {
    cdr >> ros_message->battery_voltage;
  }

  // Field name: system_voltage
  {
    cdr >> ros_message->system_voltage;
  }

  // Field name: system_temperature
  {
    cdr >> ros_message->system_temperature;
  }

  // Field name: system_mode
  {
    cdr >> ros_message->system_mode;
  }

  // Field name: system_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->system_ok = tmp ? true : false;
  }

  // Field name: laser_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->laser_ok = tmp ? true : false;
  }

  // Field name: scanner_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->scanner_ok = tmp ? true : false;
  }

  // Field name: receiver_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->receiver_ok = tmp ? true : false;
  }

  // Field name: datapath_ok
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->datapath_ok = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_luminar_iris_msgs
size_t get_serialized_size_luminar_iris_msgs__msg__SensorHealth(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SensorHealth__ros_msg_type * ros_message = static_cast<const _SensorHealth__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stamp

  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);
  // field.name battery_voltage
  {
    size_t item_size = sizeof(ros_message->battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name system_voltage
  {
    size_t item_size = sizeof(ros_message->system_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name system_temperature
  {
    size_t item_size = sizeof(ros_message->system_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name system_mode
  {
    size_t item_size = sizeof(ros_message->system_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name system_ok
  {
    size_t item_size = sizeof(ros_message->system_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name laser_ok
  {
    size_t item_size = sizeof(ros_message->laser_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scanner_ok
  {
    size_t item_size = sizeof(ros_message->scanner_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name receiver_ok
  {
    size_t item_size = sizeof(ros_message->receiver_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name datapath_ok
  {
    size_t item_size = sizeof(ros_message->datapath_ok);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SensorHealth__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_luminar_iris_msgs__msg__SensorHealth(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_luminar_iris_msgs
size_t max_serialized_size_luminar_iris_msgs__msg__SensorHealth(
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

  // member: stamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: system_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: system_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: system_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: system_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: laser_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: scanner_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: receiver_ok
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: datapath_ok
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
    using DataType = luminar_iris_msgs__msg__SensorHealth;
    is_plain =
      (
      offsetof(DataType, datapath_ok) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SensorHealth__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_luminar_iris_msgs__msg__SensorHealth(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SensorHealth = {
  "luminar_iris_msgs::msg",
  "SensorHealth",
  _SensorHealth__cdr_serialize,
  _SensorHealth__cdr_deserialize,
  _SensorHealth__get_serialized_size,
  _SensorHealth__max_serialized_size
};

static rosidl_message_type_support_t _SensorHealth__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SensorHealth,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, luminar_iris_msgs, msg, SensorHealth)() {
  return &_SensorHealth__type_support;
}

#if defined(__cplusplus)
}
#endif
