// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice
#include "luminar_iris_msgs/msg/detail/sensor_health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
luminar_iris_msgs__msg__SensorHealth__init(luminar_iris_msgs__msg__SensorHealth * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    luminar_iris_msgs__msg__SensorHealth__fini(msg);
    return false;
  }
  // battery_voltage
  // system_voltage
  // system_temperature
  // system_mode
  // system_ok
  // laser_ok
  // scanner_ok
  // receiver_ok
  // datapath_ok
  return true;
}

void
luminar_iris_msgs__msg__SensorHealth__fini(luminar_iris_msgs__msg__SensorHealth * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // battery_voltage
  // system_voltage
  // system_temperature
  // system_mode
  // system_ok
  // laser_ok
  // scanner_ok
  // receiver_ok
  // datapath_ok
}

bool
luminar_iris_msgs__msg__SensorHealth__are_equal(const luminar_iris_msgs__msg__SensorHealth * lhs, const luminar_iris_msgs__msg__SensorHealth * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // system_voltage
  if (lhs->system_voltage != rhs->system_voltage) {
    return false;
  }
  // system_temperature
  if (lhs->system_temperature != rhs->system_temperature) {
    return false;
  }
  // system_mode
  if (lhs->system_mode != rhs->system_mode) {
    return false;
  }
  // system_ok
  if (lhs->system_ok != rhs->system_ok) {
    return false;
  }
  // laser_ok
  if (lhs->laser_ok != rhs->laser_ok) {
    return false;
  }
  // scanner_ok
  if (lhs->scanner_ok != rhs->scanner_ok) {
    return false;
  }
  // receiver_ok
  if (lhs->receiver_ok != rhs->receiver_ok) {
    return false;
  }
  // datapath_ok
  if (lhs->datapath_ok != rhs->datapath_ok) {
    return false;
  }
  return true;
}

bool
luminar_iris_msgs__msg__SensorHealth__copy(
  const luminar_iris_msgs__msg__SensorHealth * input,
  luminar_iris_msgs__msg__SensorHealth * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // system_voltage
  output->system_voltage = input->system_voltage;
  // system_temperature
  output->system_temperature = input->system_temperature;
  // system_mode
  output->system_mode = input->system_mode;
  // system_ok
  output->system_ok = input->system_ok;
  // laser_ok
  output->laser_ok = input->laser_ok;
  // scanner_ok
  output->scanner_ok = input->scanner_ok;
  // receiver_ok
  output->receiver_ok = input->receiver_ok;
  // datapath_ok
  output->datapath_ok = input->datapath_ok;
  return true;
}

luminar_iris_msgs__msg__SensorHealth *
luminar_iris_msgs__msg__SensorHealth__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  luminar_iris_msgs__msg__SensorHealth * msg = (luminar_iris_msgs__msg__SensorHealth *)allocator.allocate(sizeof(luminar_iris_msgs__msg__SensorHealth), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(luminar_iris_msgs__msg__SensorHealth));
  bool success = luminar_iris_msgs__msg__SensorHealth__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
luminar_iris_msgs__msg__SensorHealth__destroy(luminar_iris_msgs__msg__SensorHealth * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    luminar_iris_msgs__msg__SensorHealth__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
luminar_iris_msgs__msg__SensorHealth__Sequence__init(luminar_iris_msgs__msg__SensorHealth__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  luminar_iris_msgs__msg__SensorHealth * data = NULL;

  if (size) {
    data = (luminar_iris_msgs__msg__SensorHealth *)allocator.zero_allocate(size, sizeof(luminar_iris_msgs__msg__SensorHealth), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = luminar_iris_msgs__msg__SensorHealth__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        luminar_iris_msgs__msg__SensorHealth__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
luminar_iris_msgs__msg__SensorHealth__Sequence__fini(luminar_iris_msgs__msg__SensorHealth__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      luminar_iris_msgs__msg__SensorHealth__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

luminar_iris_msgs__msg__SensorHealth__Sequence *
luminar_iris_msgs__msg__SensorHealth__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  luminar_iris_msgs__msg__SensorHealth__Sequence * array = (luminar_iris_msgs__msg__SensorHealth__Sequence *)allocator.allocate(sizeof(luminar_iris_msgs__msg__SensorHealth__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = luminar_iris_msgs__msg__SensorHealth__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
luminar_iris_msgs__msg__SensorHealth__Sequence__destroy(luminar_iris_msgs__msg__SensorHealth__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    luminar_iris_msgs__msg__SensorHealth__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
luminar_iris_msgs__msg__SensorHealth__Sequence__are_equal(const luminar_iris_msgs__msg__SensorHealth__Sequence * lhs, const luminar_iris_msgs__msg__SensorHealth__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!luminar_iris_msgs__msg__SensorHealth__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
luminar_iris_msgs__msg__SensorHealth__Sequence__copy(
  const luminar_iris_msgs__msg__SensorHealth__Sequence * input,
  luminar_iris_msgs__msg__SensorHealth__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(luminar_iris_msgs__msg__SensorHealth);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    luminar_iris_msgs__msg__SensorHealth * data =
      (luminar_iris_msgs__msg__SensorHealth *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!luminar_iris_msgs__msg__SensorHealth__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          luminar_iris_msgs__msg__SensorHealth__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!luminar_iris_msgs__msg__SensorHealth__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
