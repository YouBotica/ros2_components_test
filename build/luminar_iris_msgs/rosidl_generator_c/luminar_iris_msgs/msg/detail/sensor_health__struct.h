// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice

#ifndef LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__STRUCT_H_
#define LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SYSTEM_STATUS_UNKNOWN'.
enum
{
  luminar_iris_msgs__msg__SensorHealth__SYSTEM_STATUS_UNKNOWN = 0
};

/// Constant 'SYSTEM_STATUS_STANDBY'.
enum
{
  luminar_iris_msgs__msg__SensorHealth__SYSTEM_STATUS_STANDBY = 1
};

/// Constant 'SYSTEM_STATUS_ACTIVE'.
enum
{
  luminar_iris_msgs__msg__SensorHealth__SYSTEM_STATUS_ACTIVE = 2
};

/// Constant 'SYSTEM_STATUS_SHUTDOWN'.
enum
{
  luminar_iris_msgs__msg__SensorHealth__SYSTEM_STATUS_SHUTDOWN = 3
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SensorHealth in the package luminar_iris_msgs.
typedef struct luminar_iris_msgs__msg__SensorHealth
{
  builtin_interfaces__msg__Time stamp;
  float battery_voltage;
  float system_voltage;
  float system_temperature;
  uint8_t system_mode;
  bool system_ok;
  bool laser_ok;
  bool scanner_ok;
  bool receiver_ok;
  bool datapath_ok;
} luminar_iris_msgs__msg__SensorHealth;

// Struct for a sequence of luminar_iris_msgs__msg__SensorHealth.
typedef struct luminar_iris_msgs__msg__SensorHealth__Sequence
{
  luminar_iris_msgs__msg__SensorHealth * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} luminar_iris_msgs__msg__SensorHealth__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LUMINAR_IRIS_MSGS__MSG__DETAIL__SENSOR_HEALTH__STRUCT_H_
