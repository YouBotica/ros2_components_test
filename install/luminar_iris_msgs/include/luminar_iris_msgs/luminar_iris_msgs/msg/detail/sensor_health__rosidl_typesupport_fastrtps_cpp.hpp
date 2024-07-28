// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "luminar_iris_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "luminar_iris_msgs/msg/detail/sensor_health__struct.hpp"

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

#include "fastcdr/Cdr.h"

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
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  luminar_iris_msgs::msg::SensorHealth & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
get_serialized_size(
  const luminar_iris_msgs::msg::SensorHealth & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
max_serialized_size_SensorHealth(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace luminar_iris_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_luminar_iris_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, luminar_iris_msgs, msg, SensorHealth)();

#ifdef __cplusplus
}
#endif

#endif  // LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
