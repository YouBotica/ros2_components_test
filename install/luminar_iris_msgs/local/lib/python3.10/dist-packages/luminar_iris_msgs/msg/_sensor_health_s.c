// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from luminar_iris_msgs:msg/SensorHealth.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "luminar_iris_msgs/msg/detail/sensor_health__struct.h"
#include "luminar_iris_msgs/msg/detail/sensor_health__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool luminar_iris_msgs__msg__sensor_health__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("luminar_iris_msgs.msg._sensor_health.SensorHealth", full_classname_dest, 49) == 0);
  }
  luminar_iris_msgs__msg__SensorHealth * ros_message = _ros_message;
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // system_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->system_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // system_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->system_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // system_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->system_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // system_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->system_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // laser_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "laser_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->laser_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // scanner_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "scanner_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->scanner_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // receiver_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "receiver_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->receiver_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // datapath_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "datapath_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->datapath_ok = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * luminar_iris_msgs__msg__sensor_health__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SensorHealth */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("luminar_iris_msgs.msg._sensor_health");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SensorHealth");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  luminar_iris_msgs__msg__SensorHealth * ros_message = (luminar_iris_msgs__msg__SensorHealth *)raw_ros_message;
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->system_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->system_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->system_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->system_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // laser_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->laser_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "laser_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scanner_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->scanner_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scanner_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // receiver_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->receiver_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "receiver_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // datapath_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->datapath_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "datapath_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
