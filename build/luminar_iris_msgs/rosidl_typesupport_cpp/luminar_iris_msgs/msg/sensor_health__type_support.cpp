// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "luminar_iris_msgs/msg/detail/sensor_health__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace luminar_iris_msgs
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _SensorHealth_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SensorHealth_type_support_ids_t;

static const _SensorHealth_type_support_ids_t _SensorHealth_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SensorHealth_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SensorHealth_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SensorHealth_type_support_symbol_names_t _SensorHealth_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, luminar_iris_msgs, msg, SensorHealth)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, luminar_iris_msgs, msg, SensorHealth)),
  }
};

typedef struct _SensorHealth_type_support_data_t
{
  void * data[2];
} _SensorHealth_type_support_data_t;

static _SensorHealth_type_support_data_t _SensorHealth_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SensorHealth_message_typesupport_map = {
  2,
  "luminar_iris_msgs",
  &_SensorHealth_message_typesupport_ids.typesupport_identifier[0],
  &_SensorHealth_message_typesupport_symbol_names.symbol_name[0],
  &_SensorHealth_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SensorHealth_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SensorHealth_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace luminar_iris_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<luminar_iris_msgs::msg::SensorHealth>()
{
  return &::luminar_iris_msgs::msg::rosidl_typesupport_cpp::SensorHealth_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, luminar_iris_msgs, msg, SensorHealth)() {
  return get_message_type_support_handle<luminar_iris_msgs::msg::SensorHealth>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
