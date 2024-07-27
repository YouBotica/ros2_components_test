# generated from rosidl_generator_py/resource/_idl.py.em
# with input from luminar_iris_msgs:msg/SensorHealth.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SensorHealth(type):
    """Metaclass of message 'SensorHealth'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SYSTEM_STATUS_UNKNOWN': 0,
        'SYSTEM_STATUS_STANDBY': 1,
        'SYSTEM_STATUS_ACTIVE': 2,
        'SYSTEM_STATUS_SHUTDOWN': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('luminar_iris_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'luminar_iris_msgs.msg.SensorHealth')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sensor_health
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sensor_health
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sensor_health
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sensor_health
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sensor_health

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SYSTEM_STATUS_UNKNOWN': cls.__constants['SYSTEM_STATUS_UNKNOWN'],
            'SYSTEM_STATUS_STANDBY': cls.__constants['SYSTEM_STATUS_STANDBY'],
            'SYSTEM_STATUS_ACTIVE': cls.__constants['SYSTEM_STATUS_ACTIVE'],
            'SYSTEM_STATUS_SHUTDOWN': cls.__constants['SYSTEM_STATUS_SHUTDOWN'],
        }

    @property
    def SYSTEM_STATUS_UNKNOWN(self):
        """Message constant 'SYSTEM_STATUS_UNKNOWN'."""
        return Metaclass_SensorHealth.__constants['SYSTEM_STATUS_UNKNOWN']

    @property
    def SYSTEM_STATUS_STANDBY(self):
        """Message constant 'SYSTEM_STATUS_STANDBY'."""
        return Metaclass_SensorHealth.__constants['SYSTEM_STATUS_STANDBY']

    @property
    def SYSTEM_STATUS_ACTIVE(self):
        """Message constant 'SYSTEM_STATUS_ACTIVE'."""
        return Metaclass_SensorHealth.__constants['SYSTEM_STATUS_ACTIVE']

    @property
    def SYSTEM_STATUS_SHUTDOWN(self):
        """Message constant 'SYSTEM_STATUS_SHUTDOWN'."""
        return Metaclass_SensorHealth.__constants['SYSTEM_STATUS_SHUTDOWN']


class SensorHealth(metaclass=Metaclass_SensorHealth):
    """
    Message class 'SensorHealth'.

    Constants:
      SYSTEM_STATUS_UNKNOWN
      SYSTEM_STATUS_STANDBY
      SYSTEM_STATUS_ACTIVE
      SYSTEM_STATUS_SHUTDOWN
    """

    __slots__ = [
        '_stamp',
        '_battery_voltage',
        '_system_voltage',
        '_system_temperature',
        '_system_mode',
        '_system_ok',
        '_laser_ok',
        '_scanner_ok',
        '_receiver_ok',
        '_datapath_ok',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'battery_voltage': 'float',
        'system_voltage': 'float',
        'system_temperature': 'float',
        'system_mode': 'uint8',
        'system_ok': 'boolean',
        'laser_ok': 'boolean',
        'scanner_ok': 'boolean',
        'receiver_ok': 'boolean',
        'datapath_ok': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.system_voltage = kwargs.get('system_voltage', float())
        self.system_temperature = kwargs.get('system_temperature', float())
        self.system_mode = kwargs.get('system_mode', int())
        self.system_ok = kwargs.get('system_ok', bool())
        self.laser_ok = kwargs.get('laser_ok', bool())
        self.scanner_ok = kwargs.get('scanner_ok', bool())
        self.receiver_ok = kwargs.get('receiver_ok', bool())
        self.datapath_ok = kwargs.get('datapath_ok', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.stamp != other.stamp:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.system_voltage != other.system_voltage:
            return False
        if self.system_temperature != other.system_temperature:
            return False
        if self.system_mode != other.system_mode:
            return False
        if self.system_ok != other.system_ok:
            return False
        if self.laser_ok != other.laser_ok:
            return False
        if self.scanner_ok != other.scanner_ok:
            return False
        if self.receiver_ok != other.receiver_ok:
            return False
        if self.datapath_ok != other.datapath_ok:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def system_voltage(self):
        """Message field 'system_voltage'."""
        return self._system_voltage

    @system_voltage.setter
    def system_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'system_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'system_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._system_voltage = value

    @builtins.property
    def system_temperature(self):
        """Message field 'system_temperature'."""
        return self._system_temperature

    @system_temperature.setter
    def system_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'system_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'system_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._system_temperature = value

    @builtins.property
    def system_mode(self):
        """Message field 'system_mode'."""
        return self._system_mode

    @system_mode.setter
    def system_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'system_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'system_mode' field must be an unsigned integer in [0, 255]"
        self._system_mode = value

    @builtins.property
    def system_ok(self):
        """Message field 'system_ok'."""
        return self._system_ok

    @system_ok.setter
    def system_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'system_ok' field must be of type 'bool'"
        self._system_ok = value

    @builtins.property
    def laser_ok(self):
        """Message field 'laser_ok'."""
        return self._laser_ok

    @laser_ok.setter
    def laser_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'laser_ok' field must be of type 'bool'"
        self._laser_ok = value

    @builtins.property
    def scanner_ok(self):
        """Message field 'scanner_ok'."""
        return self._scanner_ok

    @scanner_ok.setter
    def scanner_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'scanner_ok' field must be of type 'bool'"
        self._scanner_ok = value

    @builtins.property
    def receiver_ok(self):
        """Message field 'receiver_ok'."""
        return self._receiver_ok

    @receiver_ok.setter
    def receiver_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'receiver_ok' field must be of type 'bool'"
        self._receiver_ok = value

    @builtins.property
    def datapath_ok(self):
        """Message field 'datapath_ok'."""
        return self._datapath_ok

    @datapath_ok.setter
    def datapath_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'datapath_ok' field must be of type 'bool'"
        self._datapath_ok = value
